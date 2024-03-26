#include "EmergencyBrake.h"
#include <geometry_msgs/Twist.h>
EmergencyBrake::EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf), nh("")
{
  EmergencyBrakePub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
}
BT::NodeStatus EmergencyBrake::tick()
{  
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0; 
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  EmergencyBrakePub.publish(msg);
  //ROS_INFO("sendMessage");
  return BT::NodeStatus::SUCCESS;
}
void EmergencyBrake::halt(){
  }