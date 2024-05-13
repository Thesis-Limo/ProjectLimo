#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace BT;

class RotateAround: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
  ros::Publisher logPub;
  std_msgs::String logInfo;
public:
  RotateAround(const std::string& name, const NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
