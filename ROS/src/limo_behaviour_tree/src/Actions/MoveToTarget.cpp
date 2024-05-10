#include "MoveToTarget.h"
#include <limo_motion_controller/MovementController.h>

MoveToTarget::MoveToTarget(const std::string& name, const NodeConfiguration& conf)
  :ActionNodeBase(name, conf)
{
  logInfo.data = "Follow the Path";

}
void MoveToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  this->client = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
}

NodeStatus MoveToTarget::tick()
/*
 * currently this is debug code need to change
*/
{
  limo_motion_controller::OverrideMotion msg;
  msg.request.speed = 0;
  msg.request.angle = __FLT_MAX__;
  msg.request.duration = 0;
  ROS_INFO("Movetotarget");
  if(client.call(msg))
  {
    this->logPub.publish(logInfo);
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}
void MoveToTarget::halt(){
  }