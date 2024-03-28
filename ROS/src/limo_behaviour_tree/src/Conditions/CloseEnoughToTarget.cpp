#include "CloseEnoughToTarget.h"
CloseEnoughToTarget::CloseEnoughToTarget(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
void CloseEnoughToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    subTarget = nh.subscribe<geometry_msgs::Point>("/goal", 100, &CloseEnoughToTarget::CallBackTarget, this);
    subPosition = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &CloseEnoughToTarget::CallBackPosition, this);
}
NodeStatus CloseEnoughToTarget::tick()
{
    if(targetPos.x >= __FLT_MAX__ || currentPos.x >= __FLT_MAX__)
        return NodeStatus::RUNNING;
    Point3D toTarget = targetPos - currentPos;

    if(toTarget.DistanceSqrt() > distanceToClose * distanceToClose)
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
}