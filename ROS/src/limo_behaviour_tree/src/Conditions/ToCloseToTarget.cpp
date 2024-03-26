#include "ToCloseToTarget.h"
ToCloseToTarget::ToCloseToTarget(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
BT::NodeStatus ToCloseToTarget::tick()
{
    return BT::NodeStatus::SUCCESS;
}