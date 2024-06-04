#include "CloseEnoughToTarget.h"

CloseEnoughToTarget::CloseEnoughToTarget(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
    subTarget = nh.subscribe<geometry_msgs::Point>("/goal", 100, &CloseEnoughToTarget::CallBackTarget, this);
    subPosition = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &CloseEnoughToTarget::CallBackPosition, this);
    distanceToClose = nh.param<float>("distanceToCloseToTarget", 1);
}
NodeStatus CloseEnoughToTarget::Tick()
/*
 * checks if the target is at a certain distance in front.
*/
{
    if(targetPos.x >= __FLT_MAX__ || currentPos.x >= __FLT_MAX__)
        return NodeStatus::FAILURE;
    Point3D toTarget = targetPos - currentPos;
    if(toTarget.DistanceSqrt() > distanceToClose * distanceToClose * speedSqr)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

void CloseEnoughToTarget::CallBackTarget(const geometry_msgs::Point::ConstPtr& msg)
{
    targetPos.x = msg->x;
    targetPos.y = msg->y;
    targetPos.z = msg->z;
}

void CloseEnoughToTarget::CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    currentPos = Point3D(msg->pose.pose.position);
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
    std::cout <<speedSqr <<"\n";
    prevPos = currentPos;
    prevTimestamp = currentTime;
}