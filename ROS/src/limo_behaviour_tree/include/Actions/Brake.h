#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>

using namespace BT;

class Brake: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
public:
  Brake(const std::string& name, const BT::NodeConfiguration& conf);
  Brake() = delete;

  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
