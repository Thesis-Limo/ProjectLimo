#include "ToCloseToTarget.h"
ToCloseToTarget::ToCloseToTarget(const std::string& name, const NodeConfiguration& conf)
    :ConditionNode(name, conf)
{}

void ToCloseToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    subTarget = nh.subscribe<geometry_msgs::Point>("/goal", 100, &ToCloseToTarget::CallBackTarget, this);
    subPosition = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &ToCloseToTarget::CallBackPosition, this);
    distanceToDecideToClose = nh.param<float>("distanceToDecideToClose",1);
}
NodeStatus ToCloseToTarget::tick()
{
    if(targetPos.x >= __FLT_MAX__ || currentPos.x >= __FLT_MAX__)
        return NodeStatus::RUNNING;

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