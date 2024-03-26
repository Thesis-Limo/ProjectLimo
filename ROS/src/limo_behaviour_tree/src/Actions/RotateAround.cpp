#include "RotateAround.h"
RotateAround::RotateAround(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}
BT::NodeStatus RotateAround::tick()
{
    return BT::NodeStatus::SUCCESS;
}
void RotateAround::halt(){
  }