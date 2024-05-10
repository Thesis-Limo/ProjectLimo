#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>

using namespace BT;

class RotateAround: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
public:
  RotateAround(const std::string& name, const NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
