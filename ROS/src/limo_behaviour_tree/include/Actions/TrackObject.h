#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include "position.hpp"

using namespace BT;

class TrackObject: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
public:
  TrackObject(const std::string& name, const NodeConfiguration& conf);

  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {OutputPort<Position3D>("goal")};
  }
  void halt() override ;
};
