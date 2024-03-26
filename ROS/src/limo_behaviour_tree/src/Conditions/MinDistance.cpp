#include "MinDistance.h"
#include <boost/foreach.hpp>

MinDistance::MinDistance(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf), nh("")
{
    sub = nh.subscribe<PointCloud>("/camera/depth/points",100, &MinDistance::CallBackPoints, this);
}
BT::NodeStatus MinDistance::tick()
{
    if(minDistance < currentDistance)
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}
void MinDistance::CallBackPoints(const PointCloud::ConstPtr&  msg)
{
    float minDistance = __FLT_MAX__;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        if(minDistance > pt.z) minDistance = pt.z;
    }
    currentDistance = minDistance;
}