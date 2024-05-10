#include <ros/ros.h>
#include "MoveBack.h"
MoveBack::MoveBack(const std::string& name, const BT::NodeConfiguration& conf)
  :ActionNodeBase(name, conf), durationGoingBack(10), speedGoingBack(0.2)
{
  logInfo.data = "slowly moving back";
  this->moveMsg.request.speed = 0;
  this->moveMsg.request.angle = __FLT_MAX__;
  this->moveMsg.request.duration = 0.8;
  this->moveMsg.request.sameSpeedStart = true;
}

void MoveBack::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
  nh = nodehandle;
  this->moveBackService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
  this->logPub = logPub;
}


NodeStatus MoveBack::tick()
{
    ROS_INFO("MoveBack");

  if(this->moveBackService.call(moveMsg))
  {
    this->logPub.publish(logInfo);
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}
void MoveBack::halt(){
  }