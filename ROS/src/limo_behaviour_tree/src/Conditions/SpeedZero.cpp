#include "SpeedZero.h"
SpeedZero::SpeedZero(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
BT::NodeStatus SpeedZero::tick()
{
    return BT::NodeStatus::SUCCESS;
}