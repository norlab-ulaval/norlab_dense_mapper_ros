#include "NodeParameters.h"
#include "norlab_dense_mapper_ros/LoadMap.h"
#include "norlab_dense_mapper_ros/SaveMap.h"
#include "norlab_dense_mapper_ros/SurfaceArray.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <memory>
#include <mutex>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef PointMatcher<float> PM;

std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<norlab_dense_mapper::DenseMapper> denseMapper;
ros::Publisher surfacePublisher;
ros::Publisher covarianceMarkersPublisher;
ros::Publisher denseMapPublisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
std::mutex idleTimeLock;
std::mutex icpOdomLock;
std::list<nav_msgs::Odometry> icpOdoms;
std::string baselinkStabilizedPostfix = "_stabilized";

void saveMap(const std::string& mapFileName)
{
    ROS_INFO("Saving map to %s", mapFileName.c_str());
    denseMapper->getMap().save(mapFileName);
}

void loadMap(const std::string& mapFileName)
{
    ROS_INFO("Loading map from %s", mapFileName.c_str());

    PM::DataPoints map = PM::DataPoints::load(mapFileName);
    int euclideanDim = params->is3D ? 3 : 2;

    if (map.getEuclideanDim() != euclideanDim)
        throw std::runtime_error("Invalid map dimension");

    denseMapper->setMap(map);
}

void mapperShutdownLoop()
{
    std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();

    while (ros::ok())
    {
        idleTimeLock.lock();

        if (lastTimeInputWasProcessed.time_since_epoch().count())
            idleTime = std::chrono::steady_clock::now() - lastTimeInputWasProcessed;

        idleTimeLock.unlock();

        if (idleTime > std::chrono::duration<float>(params->maxIdleTime))
        {
            saveMap(params->finalMapFileName);
            ROS_INFO("Shutting down ROS");
            ros::shutdown();
        }

        std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
    }
}

PM::TransformationParameters findTransform(const std::string& sourceFrame,
                                           const std::string& targetFrame,
                                           const ros::Time& time,
                                           const int& transformDimension)
{
    geometry_msgs::TransformStamped tf;

    try
    {
        tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

PM::TransformationParameters getRobotToMapTransform(const ros::Time& timeStamp,
                                                    const int& transformDimension)
{
    nav_msgs::Odometry odom1;
    nav_msgs::Odometry odom2;

    while (ros::ok())
    {
        icpOdomLock.lock();
        while (icpOdoms.size() > 1 && (++icpOdoms.begin())->header.stamp <= timeStamp)
        {
            icpOdoms.pop_front();
        }

        int icpOdomsSize = icpOdoms.size();

        if (icpOdomsSize > 1)
        {
            odom1 = icpOdoms.front();
            odom2 = *(++icpOdoms.begin());
        }
        icpOdomLock.unlock();

        if (icpOdomsSize > 1)
            break;
        else
            std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
    }

    ros::Time t1 = odom1.header.stamp;
    ros::Time t2 = odom2.header.stamp;

    geometry_msgs::Pose pose1 = odom1.pose.pose;
    geometry_msgs::Pose pose2 = odom2.pose.pose;

    tf2::Vector3 p1(pose1.position.x, pose1.position.y, pose1.position.z);
    tf2::Vector3 p2(pose2.position.x, pose2.position.y, pose2.position.z);

    tf2::Quaternion q1;
    tf2::fromMsg(pose1.orientation, q1);
    tf2::Quaternion q2;
    tf2::fromMsg(pose2.orientation, q2);

    double t = (timeStamp - t1).toSec() / (t2 - t1).toSec();

    geometry_msgs::Vector3 position(tf2::toMsg(p1.lerp(p2, t)));
    geometry_msgs::Quaternion orientation(tf2::toMsg(q1.slerp(q2, t)));

    geometry_msgs::TransformStamped tf;
    tf.transform.translation = position;
    tf.transform.rotation = orientation;

    return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

void gotInput(const PM::DataPoints& input,
              const std::string& sensorFrame,
              const ros::Time& timeStamp)
{
    PM::TransformationParameters sensorToRobot =
        findTransform(sensorFrame, params->robotFrame, timeStamp, input.getHomogeneousDim());

    PM::TransformationParameters robotToRobotStabilized =
        findTransform(params->robotFrame,
                      params->robotFrame + baselinkStabilizedPostfix,
                      timeStamp,
                      input.getHomogeneousDim());

    PM::TransformationParameters robotToMap =
        getRobotToMapTransform(timeStamp, input.getHomogeneousDim());

    PM::TransformationParameters robotStabilizedToMap =
        (robotToRobotStabilized * robotToMap.inverse()).inverse();

    denseMapper->processInput(sensorFrame,
                              input,
                              sensorToRobot,
                              robotToRobotStabilized,
                              robotStabilizedToMap,
                              std::chrono::time_point<std::chrono::steady_clock>(
                                  std::chrono::nanoseconds(timeStamp.toNSec())));

    idleTimeLock.lock();
    lastTimeInputWasProcessed = std::chrono::steady_clock::now();
    idleTimeLock.unlock();
}

void pointCloud2Callback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsgIn),
             cloudMsgIn.header.frame_id,
             cloudMsgIn.header.stamp);
}

void laserScanCallback(const sensor_msgs::LaserScan& scanMsgIn)
{
    gotInput(PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(scanMsgIn),
             scanMsgIn.header.frame_id,
             scanMsgIn.header.stamp);
}

void icpOdomCallback(const nav_msgs::Odometry& msg)
{
    icpOdomLock.lock();
    icpOdoms.emplace_back(msg);
    icpOdomLock.unlock();
}

bool reloadYamlConfigCallback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
{
    ROS_INFO_STREAM("Reloading YAML config");

    denseMapper->loadYamlConfig(params->depthCameraFiltersConfig,
                                params->lidarFiltersConfig,
                                params->robotFiltersConfig,
                                params->robotStabilizedFiltersConfig,
                                params->mapPostFiltersConfig);
    return true;
}

bool saveMapCallback(norlab_dense_mapper_ros::SaveMap::Request& request,
                     norlab_dense_mapper_ros::SaveMap::Response& response)
{
    try
    {
        saveMap(request.map_file_name.data);
        return true;
    }
    catch (const std::runtime_error& e)
    {
        ROS_ERROR_STREAM("Unable to save: " << e.what());
        return false;
    }
}

bool loadMapCallback(norlab_dense_mapper_ros::LoadMap::Request& request,
                     norlab_dense_mapper_ros::LoadMap::Response& response)
{
    try
    {
        loadMap(request.map_file_name.data);
        return true;
    }
    catch (const std::runtime_error& e)
    {
        ROS_ERROR_STREAM("Unable to load: " << e.what());
        return false;
    }
}

bool enableMappingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Enabling mapping");
    denseMapper->setIsMapping(true);
    return true;
}

bool disableMappingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Disabling mapping");
    denseMapper->setIsMapping(false);
    return true;
}

void denseMapPublisherLoop()
{
    ros::Rate publishRate(params->mapPublishRate);

    PM::DataPoints newMap;

    while (ros::ok())
    {
        if (denseMapper->getNewLocalMap(newMap))
        {
            sensor_msgs::PointCloud2 dense_map = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
                newMap, params->mapFrame, ros::Time::now());

            PM::Matrix eigenValues =
                newMap.getDescriptorCopyByName("eigValues").unaryExpr([](float element) {
                    return element < 1e-6 ? 0.001f : element;
                });
            PM::Matrix eigenVectors = newMap.getDescriptorCopyByName("eigVectors");
            PM::Matrix nbPoints = newMap.getDescriptorCopyByName("nbPoints");

            norlab_dense_mapper_ros::SurfaceArray surfaceArray;
            visualization_msgs::MarkerArray markers;
            for (size_t i = 0; i < newMap.getNbPoints(); ++i)
            {
                const auto& a = eigenVectors.block<3, 1>(0, i);
                const auto& b = eigenVectors.block<3, 1>(3, i);
                const auto& c = eigenVectors.block<3, 1>(6, i);

                if (params->isSurfacePublisherEnabled)
                {
                    // only one surface per prism (the top one)
                    Eigen::Vector3f topEigenVector, edge1, edge2;
                    if (std::fabs(a(2)) > std::fabs(b(2)))
                    {
                        if (std::fabs(a(2)) > std::fabs(c(2)))
                        {
                            topEigenVector = a * std::sqrt(3 * eigenValues(0, i));
                            edge1 = b * std::sqrt(3 * eigenValues(1, i));
                            edge2 = c * std::sqrt(3 * eigenValues(2, i));
                        }
                        else
                        {
                            edge1 = a * std::sqrt(3 * eigenValues(0, i));
                            edge2 = b * std::sqrt(3 * eigenValues(1, i));
                            topEigenVector = c * std::sqrt(3 * eigenValues(2, i));
                        }
                    }
                    else
                    {
                        if (std::fabs(b(2)) > std::fabs(c(2)))
                        {
                            edge1 = a * std::sqrt(3 * eigenValues(0, i));
                            topEigenVector = b * std::sqrt(3 * eigenValues(1, i));
                            edge2 = c * std::sqrt(3 * eigenValues(2, i));
                        }
                        else
                        {
                            edge1 = a * std::sqrt(3 * eigenValues(0, i));
                            edge2 = b * std::sqrt(3 * eigenValues(1, i));
                            topEigenVector = c * std::sqrt(3 * eigenValues(2, i));
                        }
                    }
                    if (topEigenVector(2) < 0)
                    {
                        topEigenVector *= -1;
                    }
                    Eigen::Vector3f center = newMap.features.col(i).topRows(3) + topEigenVector;

                    norlab_dense_mapper_ros::Surface surface;
                    surface.center.x = center(0);
                    surface.center.y = center(1);
                    surface.center.z = center(2);
                    surface.edge_1.x = edge1(0);
                    surface.edge_1.y = edge1(1);
                    surface.edge_1.z = edge1(2);
                    surface.edge_2.x = edge2(0);
                    surface.edge_2.y = edge2(1);
                    surface.edge_2.z = edge2(2);
                    surfaceArray.surfaces.emplace_back(surface);
                }

                if (params->isCovarianceMarkersEnabled)
                {
                    Eigen::Matrix3f R;
                    R << a, b, c;

                    if (R.determinant() < 0)
                    {
                        R.col(0).swap(R.col(1));
                        eigenValues.row(0).swap(eigenValues.row(1));
                    }

                    Eigen::Quaternionf q(R);
                    q.normalize();

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = params->mapFrame;
                    marker.header.stamp = dense_map.header.stamp;
                    marker.ns = "covariances";
                    marker.id = i;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = newMap.features(0, i);
                    marker.pose.position.y = newMap.features(1, i);
                    marker.pose.position.z = newMap.features(2, i);
                    marker.pose.orientation.x = q.x();
                    marker.pose.orientation.y = q.y();
                    marker.pose.orientation.z = q.z();
                    marker.pose.orientation.w = q.w();
                    marker.scale.x = 2 * std::sqrt(3 * eigenValues(0, i));
                    marker.scale.y = 2 * std::sqrt(3 * eigenValues(1, i));
                    marker.scale.z = 2 * std::sqrt(3 * eigenValues(2, i));
                    marker.color.r = (float)std::min(nbPoints(0, i) / 500.0, 1.0);
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f - (float)std::min(nbPoints(0, i) / 500.0, 1.0);
                    marker.color.a = 0.75f;
                    markers.markers.emplace_back(marker);
                }
            }

            if (params->isSurfacePublisherEnabled)
                surfacePublisher.publish(surfaceArray);

            if (params->isCovarianceMarkersEnabled)
                covarianceMarkersPublisher.publish(markers);

            denseMapPublisher.publish(dense_map);
        }
        publishRate.sleep();
    }
}

void generatePointCloud()
{
    unsigned int numberOfPoints = 10000;
    float halfWidth = 1.0;
    float halfLength = 1.5;
    float halfThickness = 0.015;
    float height = -0.12;

    PM::Matrix randomFeatures(PM::Matrix::Random(4, numberOfPoints));
    randomFeatures.row(0) *= halfLength;
    randomFeatures.row(1) *= halfWidth;
    randomFeatures.row(2) = (randomFeatures.row(2) * halfThickness).array() + height;
    randomFeatures.row(3) = PM::Matrix::Ones(1, numberOfPoints);

    PM::DataPoints::Labels featuresLabels;
    featuresLabels.emplace_back("x", 1);
    featuresLabels.emplace_back("y", 1);
    featuresLabels.emplace_back("z", 1);
    featuresLabels.emplace_back("pad", 1);
    PM::DataPoints pointCloud(randomFeatures, featuresLabels);

    denseMapper->processInput(params->robotFrame + baselinkStabilizedPostfix,
                              pointCloud,
                              PM::Matrix::Identity(4, 4),
                              PM::Matrix::Identity(4, 4),
                              PM::Matrix::Identity(4, 4),
                              std::chrono::time_point<std::chrono::steady_clock>(
                                  std::chrono::nanoseconds(ros::Time::now().toNSec())));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dense_mapper_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));
    transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
    denseMapPublisher = n.advertise<sensor_msgs::PointCloud2>("dense_map", 10, true);

    if (params->isSurfacePublisherEnabled)
        surfacePublisher = n.advertise<norlab_dense_mapper_ros::SurfaceArray>("surfaces", 1, true);

    if (params->isCovarianceMarkersEnabled)
        covarianceMarkersPublisher =
            n.advertise<visualization_msgs::MarkerArray>("covariance_markers", 10, true);

    denseMapper = std::unique_ptr<norlab_dense_mapper::DenseMapper>(
        new norlab_dense_mapper::DenseMapper(params->depthCameraFiltersConfig,
                                             params->lidarFiltersConfig,
                                             params->robotFiltersConfig,
                                             params->robotStabilizedFiltersConfig,
                                             params->mapPostFiltersConfig,
                                             params->mapUpdateCondition,
                                             params->depthCameraFrame,
                                             params->mapUpdateDelay,
                                             params->mapUpdateDistance,
                                             params->minDistNewPoint,
                                             params->sensorMaxRange,
                                             params->priorDynamic,
                                             params->thresholdDynamic,
                                             params->beamHalfAngle,
                                             params->epsilonA,
                                             params->epsilonD,
                                             params->alpha,
                                             params->beta,
                                             params->is3D,
                                             params->isDepthCameraEnabled,
                                             params->isOnline,
                                             params->isMapping,
                                             params->computeProbDynamic,
                                             params->saveMapCellsOnHardDrive));

    if (!params->initialMapFileName.empty())
        loadMap(params->initialMapFileName);

    std::thread mapperShutdownThread;
    int messageQueueSize;

    if (params->isOnline)
    {
        tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
        messageQueueSize = 10;
    }
    else
    {
        mapperShutdownThread = std::thread(mapperShutdownLoop);
        tfBuffer =
            std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(ros::DURATION_MAX)));
        messageQueueSize = 0;
    }

    tf2_ros::TransformListener tfListener(*tfBuffer);
    ros::Subscriber icpOdomSubscriber(n.subscribe("icp_odom", 10, icpOdomCallback));
    ros::Subscriber sensorsSubscriber;

    if (params->is3D)
        sensorsSubscriber =
            n.subscribe("lslidar_point_cloud_deskewed", messageQueueSize, pointCloud2Callback);
    else
        sensorsSubscriber =
            n.subscribe("lslidar_point_cloud_deskewed", messageQueueSize, laserScanCallback);

    ros::ServiceServer reloadYamlConfigService =
        n.advertiseService("reload_yaml_config", reloadYamlConfigCallback);
    ros::ServiceServer saveMapService = n.advertiseService("save_map", saveMapCallback);
    ros::ServiceServer loadMapService = n.advertiseService("load_map", loadMapCallback);
    ros::ServiceServer enableMappingService =
        n.advertiseService("enable_mapping", enableMappingCallback);
    ros::ServiceServer disableMappingService =
        n.advertiseService("disable_mapping", disableMappingCallback);

    generatePointCloud();

    std::thread denseMapPublisherThread = std::thread(denseMapPublisherLoop);
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    denseMapPublisherThread.join();

    if (!params->isOnline)
        mapperShutdownThread.join();

    return 0;
}
