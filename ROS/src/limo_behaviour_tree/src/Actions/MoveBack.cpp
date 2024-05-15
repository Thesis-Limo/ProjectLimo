#include "MoveBack.h"

MoveBack::MoveBack(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub,  "slowly moving back"), durationGoingBack(10), speedGoingBack(0.2)
{
  this->MoveBackService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
  this->moveMsg.request.speed = nh.param<float>("SpeedGoingBack",-0.1);;
  this->moveMsg.request.angle = __FLT_MAX__;
  this->moveMsg.request.duration = -1;
  this->moveMsg.request.sameSpeedStart = true;
}

NodeStatus MoveBack::Tick()
{
  if(this->MoveBackService.call(moveMsg))
  {
    Log();
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}