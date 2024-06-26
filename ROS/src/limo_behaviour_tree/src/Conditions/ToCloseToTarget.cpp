#include "ToCloseToTarget.h"

ToCloseToTarget::ToCloseToTarget(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
    subTarget = nh.subscribe<geometry_msgs::Point>("/ObjectPos", 100, &ToCloseToTarget::CallBackTarget, this);
    subPosition = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &ToCloseToTarget::CallBackPosition, this);
    distanceToDecideToClose = nh.param<float>("distanceToDecideToClose",1);
}
NodeStatus ToCloseToTarget::Tick()
/*
 * checks if the target is to close to the object or not based on distance
*/
{
    if(targetPos.x >= __FLT_MAX__ || currentPos.x >= __FLT_MAX__)
        return NodeStatus::FAILURE;

    Point3D toTarget = targetPos - currentPos;
    
    if(toTarget.DistanceSqrt() < distanceToDecideToClose * distanceToDecideToClose)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

void ToCloseToTarget::CallBackTarget(const geometry_msgs::Point::ConstPtr& msg)
{
    targetPos.x = msg->x;
    targetPos.y = msg->y;
    targetPos.z = msg->z;
}

void ToCloseToTarget::CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPos = Point3D(msg->pose.pose.position);
}