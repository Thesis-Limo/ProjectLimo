#include "CloseEnoughToTarget.h"
CloseEnoughToTarget::CloseEnoughToTarget(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
BT::NodeStatus CloseEnoughToTarget::tick()
{
    return BT::NodeStatus::SUCCESS;
}