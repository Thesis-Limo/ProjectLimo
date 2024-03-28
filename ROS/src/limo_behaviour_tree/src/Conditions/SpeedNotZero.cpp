#include "SpeedNotZero.h"

SpeedNotZero::SpeedNotZero(const std::string& name, const NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{
    prevPos.x = __FLT_MAX__;
}
void SpeedNotZero::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    sub = nh.subscribe<nav_msgs::Odometry>("/odom",100, &SpeedNotZero::CallBackOdom, this);
}

NodeStatus SpeedNotZero::tick()
{
    ROS_INFO("not suppose to be");
    if(speedSqr > 0)
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}

void SpeedNotZero::CallBackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::Time currentTime = msg->header.stamp;
    geometry_msgs::Point currentPos = msg->pose.pose.position;
    if (prevPos.x >= __FLT_MAX__)
    {
        prevPos = currentPos;
        prevTimestamp = currentTime;
        return;
    }
    geometry_msgs::Point trans = currentPos;
    trans.x -= prevPos.x;
    trans.y -= prevPos.y;
    trans.z -= prevPos.z;
    double t = (currentTime - prevTimestamp).toSec();

    trans.x /= t;
    trans.y /= t;
    trans.z /= t;
    speedSqr = trans.x*trans.x + trans.y*trans.y + trans.z* trans.z;
    prevPos = currentPos;
    prevTimestamp = currentTime;
}