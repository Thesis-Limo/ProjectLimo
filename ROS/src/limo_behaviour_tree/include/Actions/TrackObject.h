#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>

using namespace BT;

class TrackObject: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
  int currentTargetID = 0;
public:
  TrackObject(const std::string& name, const NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {};
  }
  void halt() override ;
};
