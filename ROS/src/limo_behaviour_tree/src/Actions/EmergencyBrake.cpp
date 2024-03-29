#include "EmergencyBrake.h"
#include <limo_motion_controller/movementController.h>
EmergencyBrake::EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{
}
void EmergencyBrake::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  EmergencyBrakePub = nh.advertise<limo_motion_controller::movementController>("/limo_movement", 100);
}

NodeStatus EmergencyBrake::tick()
{ 
  limo_motion_controller::movementController msg;
  msg.speed = 0;
  msg.angle = 0;
  EmergencyBrakePub.publish(msg);
  ROS_INFO("emergency brake");
  return NodeStatus::SUCCESS;
}
void EmergencyBrake::halt(){
  }