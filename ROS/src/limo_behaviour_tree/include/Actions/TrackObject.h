#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
using namespace BT;

class TrackObject: public ActionNodeBase
{
public:
  TrackObject(const std::string& name, const BT::NodeConfiguration& conf);

  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
