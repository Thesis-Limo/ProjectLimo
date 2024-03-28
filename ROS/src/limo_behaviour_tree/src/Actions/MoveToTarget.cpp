#include "MoveToTarget.h"
MoveToTarget::MoveToTarget(const std::string& name, const NodeConfiguration& conf)
  :ActionNodeBase(name, conf)
{}
void MoveToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  movePub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

NodeStatus MoveToTarget::tick()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.3;
  msg.linear.y = 0;
  msg.linear.z = 0; 
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  movePub.publish(msg);
  ROS_INFO("MoveTowardsTarget");
  return NodeStatus::SUCCESS;
}
void MoveToTarget::halt(){
  }