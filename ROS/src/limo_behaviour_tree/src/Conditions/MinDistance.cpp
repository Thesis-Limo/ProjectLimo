#include "MinDistance.h"
#include <boost/foreach.hpp>

MinDistance::MinDistance(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf), nh("")
{
    sub = nh.subscribe<geometry_msgs::PoseArray>("/particlecloud",100, &MinDistance::CallBackPoints, this);
}
BT::NodeStatus MinDistance::tick()
{
    return BT::NodeStatus::SUCCESS;
}
void MinDistance::CallBackPoints(const  geometry_msgs::PoseArray::ConstPtr&  msg)
{
    ROS_INFO("runThishit");
    float minDistance = __FLT_MAX__;
    for(int i = 0; i < int(msg->poses.size()); i++) {
        if(minDistance > msg->poses[i].position.z) minDistance = msg->poses[i].position.z;
    }
    std::cout << minDistance;
}