#include "SpeedNotZero.h"

SpeedNotZero::SpeedNotZero(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
    prevPos.x = __FLT_MAX__;
    sub = nh.subscribe<nav_msgs::Odometry>("/odom",100, &SpeedNotZero::CallBackOdom, this);
}

NodeStatus SpeedNotZero::Tick()
/*
 * checks if speed is more then 0 or not
*/
{
    ROS_INFO("speedNotZero %f", speedSqr);
    if(speedSqr > 0)
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}

void SpeedNotZero::CallBackOdom(const nav_msgs::Odometry::ConstPtr& msg)
/*
 * checks the current position and updates the current velocity based on the previous position
*/
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