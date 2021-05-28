#include "NodeParameters.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <std_srvs/Empty.h>
#include "norlab_dense_mapper_ros/SaveMap.h"
#include "norlab_dense_mapper_ros/LoadMap.h"
#include <geometry_msgs/Pose.h>
#include <memory>
#include <mutex>
#include <thread>

typedef PointMatcher<float> PM;

std::unique_ptr<NodeParameters> params;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<norlab_dense_mapper::DenseMapper> denseMapper;
ros::Publisher mapPublisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::chrono::time_point<std::chrono::steady_clock> lastTimeInputWasProcessed;
std::mutex idleTimeLock;
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
    {
        throw std::runtime_error("Invalid map dimension");
    }
    denseMapper->setMap(map);
}

void mapperShutdownLoop()
{
    std::chrono::duration<float> idleTime = std::chrono::duration<float>::zero();

    while (ros::ok())
    {
        idleTimeLock.lock();

        if (lastTimeInputWasProcessed.time_since_epoch().count())
        {
            idleTime = std::chrono::steady_clock::now() - lastTimeInputWasProcessed;
        }

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

    PM::TransformationParameters robotStabilizedToMap =
        findTransform(params->robotFrame + baselinkStabilizedPostfix,
                      params->mapFrame,
                      timeStamp,
                      input.getHomogeneousDim());

    denseMapper->processInput(input,
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

bool reloadYamlConfigCallback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
{
    ROS_INFO("Reloading YAML config");
    denseMapper->loadYamlConfig(params->sensorFiltersConfig,
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

void mapPublisherLoop()
{
    ros::Rate publishRate(params->mapPublishRate);

    PM::DataPoints newMap;

    while (ros::ok())
    {
        if (denseMapper->getNewLocalMap(newMap))
        {
            sensor_msgs::PointCloud2 mapMsgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
                newMap, params->mapFrame, ros::Time::now());
            mapPublisher.publish(mapMsgOut);
        }

        publishRate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dense_mapper_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));
    transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

    denseMapper = std::unique_ptr<norlab_dense_mapper::DenseMapper>(
        new norlab_dense_mapper::DenseMapper(params->sensorFiltersConfig,
                                             params->robotFiltersConfig,
                                             params->robotStabilizedFiltersConfig,
                                             params->mapPostFiltersConfig,
                                             params->mapUpdateCondition,
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
                                             params->isOnline,
                                             params->computeProbDynamic,
                                             params->isMapping,
                                             params->saveMapCellsOnHardDrive));

    if (!params->initialMapFileName.empty())
    {
        loadMap(params->initialMapFileName);
    }

    std::thread mapperShutdownThread;
    int messageQueueSize;

    if (params->isOnline)
    {
        tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
        messageQueueSize = 1;
    }
    else
    {
        mapperShutdownThread = std::thread(mapperShutdownLoop);
        tfBuffer =
            std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(ros::DURATION_MAX)));
        messageQueueSize = 0;
    }

    tf2_ros::TransformListener tfListener(*tfBuffer);
    ros::Subscriber sub;

    if (params->is3D)
    {
        sub = n.subscribe("points_in", messageQueueSize, pointCloud2Callback);
    }
    else
    {
        sub = n.subscribe("points_in", messageQueueSize, laserScanCallback);
    }

    mapPublisher = n.advertise<sensor_msgs::PointCloud2>("dense_map", 2, true);

    ros::ServiceServer reloadYamlConfigService =
        n.advertiseService("reload_yaml_config", reloadYamlConfigCallback);
    ros::ServiceServer saveMapService = n.advertiseService("save_map", saveMapCallback);
    ros::ServiceServer loadMapService = n.advertiseService("load_map", loadMapCallback);
    ros::ServiceServer enableMappingService =
        n.advertiseService("enable_mapping", enableMappingCallback);
    ros::ServiceServer disableMappingService =
        n.advertiseService("disable_mapping", disableMappingCallback);

    std::thread mapPublisherThread = std::thread(mapPublisherLoop);

    ros::spin();

    mapPublisherThread.join();

    if (!params->isOnline)
    {
        mapperShutdownThread.join();
    }

    return 0;
}
