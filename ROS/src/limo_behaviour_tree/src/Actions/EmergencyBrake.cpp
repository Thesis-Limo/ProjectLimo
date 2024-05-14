#include "EmergencyBrake.h"
#include <limo_motion_controller/MovementController.h>
EmergencyBrake::EmergencyBrake(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub)
    :Node(nodehandle, logPub,  "Something is in front I need to brake right now")
{
  this->brakeService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
  currentRate = duration;
}

NodeStatus EmergencyBrake::Tick()
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
    Log();
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}