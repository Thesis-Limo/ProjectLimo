#include "SpeedNotZero.h"
SpeedNotZero::SpeedNotZero(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{

}
BT::NodeStatus SpeedNotZero::tick()
{
    return BT::NodeStatus::SUCCESS;
}