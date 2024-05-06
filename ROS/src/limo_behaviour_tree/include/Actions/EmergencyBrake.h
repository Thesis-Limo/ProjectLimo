#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace BT;

class EmergencyBrake: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::Publisher EmergencyBrakePub;
  float currentRate;
public:
  EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, float duration);
  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
