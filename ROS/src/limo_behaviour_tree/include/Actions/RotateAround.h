#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>

using namespace BT;

class RotateAround: public ActionNodeBase
{
public:
  RotateAround(const std::string& name, const NodeConfiguration& conf);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
