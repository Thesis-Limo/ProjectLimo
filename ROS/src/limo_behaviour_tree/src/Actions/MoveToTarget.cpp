#include "MoveToTarget.h"
#include <limo_motion_controller/MovementController.h>

MoveToTarget::MoveToTarget(const std::string& name, const NodeConfiguration& conf)
  :ActionNodeBase(name, conf)
{}
void MoveToTarget::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  movePub = nh.advertise<limo_motion_controller::MovementController>("/limo_movement", 1);
}

NodeStatus MoveToTarget::tick()
/*
 * currently this is debug code need to change
*/
{
  limo_motion_controller::MovementController msg;
  msg.speed = 0.3;
  msg.angle = 0;
  movePub.publish(msg);
  std::cout << "move\n";
  return NodeStatus::SUCCESS;
}
void MoveToTarget::halt(){
  }