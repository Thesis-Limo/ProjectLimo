#include "MoveToTarget.h"
#include <limo_motion_controller/movementController.h>

MoveToTarget::MoveToTarget(const std::string& name, const NodeConfiguration& conf)
  :ActionNodeBase(name, conf)
{}
void MoveToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  movePub = nh.advertise<limo_motion_controller::movementController>("/limo_movement", 1);
}

NodeStatus MoveToTarget::tick()
{
  limo_motion_controller::movementController msg;
  msg.speed = 0.3;
  msg.angle = 0;
  movePub.publish(msg);
  std::cout << "move\n";
  return NodeStatus::SUCCESS;
}
void MoveToTarget::halt(){
  }