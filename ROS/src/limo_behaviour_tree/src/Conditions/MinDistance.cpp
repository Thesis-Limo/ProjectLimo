#include "MinDistance.h"
#include <boost/foreach.hpp>

MinDistance::MinDistance(const std::string& name, const NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{
}
void MinDistance::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    sub = nh.subscribe<PointCloud>("/camera/depth/points",100, &MinDistance::CallBackPoints, this);
    minDistance = nh.param<float>("minDistanceWhenBraking",0.25f);
}
NodeStatus MinDistance::tick()
/*
 * if the current distance is to close to the object then return success
*/
{
    if(currentDistance < minDistance)
        return NodeStatus::SUCCESS;
    //std::cout << currentDistance <<"\n";
    return NodeStatus::FAILURE;
}
void MinDistance::CallBackPoints(const PointCloud::ConstPtr&  msg)
/*
 * checks every pixel from the depth camera and looks for closest distance
*/
{
    float minDistance = __FLT_MAX__;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        if(minDistance > pt.z) minDistance = pt.z;
    }
    currentDistance = minDistance;
}