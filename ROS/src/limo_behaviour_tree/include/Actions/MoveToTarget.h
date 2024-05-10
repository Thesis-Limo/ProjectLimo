#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <limo_motion_controller/OverrideMotion.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

using namespace BT;
class MoveToTarget: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::Publisher movePub;
  ros::Publisher logPub;
  std_msgs::String logInfo;
  ros::ServiceClient client;
public:
  MoveToTarget(const std::string& name, const NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {};
  }
  void halt() override ;
};
