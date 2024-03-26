#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"

using namespace BT;

class SpeedZero: public ConditionNode
{
private:

public:
    SpeedZero(const std::string& name, const BT::NodeConfiguration& conf);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
};