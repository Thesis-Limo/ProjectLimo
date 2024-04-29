#include "RotateAround.h"
#include <geometry_msgs/Twist.h>
RotateAround::RotateAround(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}

BT::NodeStatus RotateAround::tick()
/*
 * This function does at the moment nothing
*/
{
  return NodeStatus::SUCCESS;   
}
void RotateAround::halt(){
  }