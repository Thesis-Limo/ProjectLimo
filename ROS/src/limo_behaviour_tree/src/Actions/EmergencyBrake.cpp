#include "EmergencyBrake.h"
#include <limo_motion_controller/MovementController.h>
EmergencyBrake::EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf), currentRate(4)
{
  logInfo.data = "Something is in front I need to brake right now";
}
void EmergencyBrake::Initialize(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub)
{
  nh = nodehandle;
  this->brakeService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
  currentRate = duration;
  this->logPub = logPub;
}

NodeStatus EmergencyBrake::tick()
/*
 * Calls the movement controller with current speed and angle velocity, to (0,0)
*/
{ 
  limo_motion_controller::OverrideMotion msg;
  msg.request.speed = 0;
  msg.request.angle = __FLT_MAX__;
  msg.request.duration = 1;
  msg.request.sameSpeedStart = true;
  if(brakeService.call(msg))
  {
    ROS_INFO("BRAAAAKE");
    this->logPub.publish(logInfo);
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}
void EmergencyBrake::halt(){
  }