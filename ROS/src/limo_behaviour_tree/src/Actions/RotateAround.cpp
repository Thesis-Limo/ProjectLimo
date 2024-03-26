#include "RotateAround.h"
#include <geometry_msgs/Twist.h>
RotateAround::RotateAround(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}
BT::NodeStatus RotateAround::tick()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0; 
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  //EmergencyBrakePub.publish(msg);
  ROS_INFO("Rotate around");
  return NodeStatus::SUCCESS;
    
}
void RotateAround::halt(){
  }