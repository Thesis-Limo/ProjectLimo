#include "MoveToTarget.h"
#include <limo_motion_controller/MovementController.h>

MoveToTarget::MoveToTarget(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub, "Follow the Path")
{
  this->client = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
}

NodeStatus MoveToTarget::Tick()
/*
 * remove the overrride funciton
*/
{
  limo_motion_controller::OverrideMotion msg;
  msg.request.speed = 0;
  msg.request.angle = __FLT_MAX__;
  msg.request.duration = 0;
  if(client.call(msg))
  {
    Log();
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}