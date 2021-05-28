#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

#include <norlab_dense_mapper/DenseMapper.h>
#include <ros/ros.h>

class NodeParameters
{
  private:
    typedef PointMatcher<float> PM;

    void retrieveParameters(const ros::NodeHandle& nodeHandle);
    void validateParameters() const;

  public:
    std::string mapFrame;
    std::string robotFrame;
    std::string initialMapFileName;
    std::string finalMapFileName;
    std::string sensorFiltersConfig;
    std::string robotFiltersConfig;
    std::string robotStabilizedFiltersConfig;
    std::string mapPostFiltersConfig;
    std::string mapUpdateCondition;
    float mapUpdateDelay;
    float mapUpdateDistance;
    float mapPublishRate;
    float maxIdleTime;
    float minDistNewPoint;
    float sensorMaxRange;
    float priorDynamic;
    float thresholdDynamic;
    float beamHalfAngle;
    float epsilonA;
    float epsilonD;
    float alpha;
    float beta;
    bool is3D;
    bool isOnline;
    bool computeProbDynamic;
    bool isMapping;
    bool saveMapCellsOnHardDrive;

    NodeParameters(ros::NodeHandle privateNodeHandle);
};

#endif
