#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace BT;
class MoveToTarget: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::Publisher movePub;
public:
  MoveToTarget(const std::string& name, const NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {InputPort<std::string>("message")};
  }
  void halt() override ;
};
