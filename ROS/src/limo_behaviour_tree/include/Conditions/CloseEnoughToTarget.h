#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

using namespace BT;

class CloseEnoughToTarget: public ConditionNode
{
public:
    CloseEnoughToTarget(const std::string& name, const BT::NodeConfiguration& conf);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
};