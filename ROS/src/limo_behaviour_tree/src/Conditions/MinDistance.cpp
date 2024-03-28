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
}
NodeStatus MinDistance::tick()
{
    if(currentDistance < minDistance)
        return NodeStatus::SUCCESS;
    //std::cout << currentDistance <<"\n";
    return NodeStatus::FAILURE;
}
void MinDistance::CallBackPoints(const PointCloud::ConstPtr&  msg)
{
    float minDistance = __FLT_MAX__;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        if(minDistance > pt.z) minDistance = pt.z;
    }
    currentDistance = minDistance;
}