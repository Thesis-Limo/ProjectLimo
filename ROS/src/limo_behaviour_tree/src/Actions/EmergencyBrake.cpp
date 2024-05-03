#include "EmergencyBrake.h"
#include <limo_motion_controller/MovementController.h>
EmergencyBrake::EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{
}
void EmergencyBrake::Initialize(const ros::NodeHandle& nodehandle, const ros::Rate rate)
{
  nh = nodehandle;
  EmergencyBrakePub = nh.advertise<limo_motion_controller::MovementController>("/limo_movement", 100);
}

NodeStatus EmergencyBrake::tick()
/*
 * Calls the movement controller with current speed and angle velocity, to (0,0)
*/
{ 
  limo_motion_controller::MovementController msg;
  msg.speed = 0;
  msg.angle = 0;
  msg.duration = currentRate.cycleTime();
  EmergencyBrakePub.publish(msg);
  ROS_INFO("emergency brake");
  return NodeStatus::SUCCESS;
}
void EmergencyBrake::halt(){
  }