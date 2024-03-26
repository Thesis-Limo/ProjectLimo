#include "behaviortree_cpp_v3/condition_node.h"

using namespace BT;

class ToCloseToTarget: public ConditionNode
{
private:

public:
    ToCloseToTarget(const std::string& name, const BT::NodeConfiguration& conf);
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts(){return {};}
};