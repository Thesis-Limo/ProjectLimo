#include "MoveToTarget.h"
MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}
BT::NodeStatus MoveToTarget::tick()
{
    return BT::NodeStatus::SUCCESS;
}
void MoveToTarget::halt(){
  }