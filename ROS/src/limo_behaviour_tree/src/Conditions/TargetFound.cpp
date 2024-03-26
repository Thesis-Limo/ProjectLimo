#include "TargetFound.h"
TargetFound::TargetFound(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
BT::NodeStatus TargetFound::tick()
{
    return BT::NodeStatus::SUCCESS;
}