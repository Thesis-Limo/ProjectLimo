#include "SpeedZero.h"
SpeedZero::SpeedZero(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
    prevPos.x = __FLT_MAX__;
    sub = nh.subscribe<nav_msgs::Odometry>("/odom",100, &SpeedZero::CallBackOdom, this);
}
NodeStatus SpeedZero::Tick()
/*
 * checks if velocity is more then 0 or not, if it is 0 return SUCCESS
*/
{
    ROS_INFO("speedzero %f", speedSqr);
    if(speedSqr <= 0.01)
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}
void SpeedZero::CallBackOdom(const nav_msgs::Odometry::ConstPtr& msg)
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