#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <limo_motion_controller/OverrideMotion.h>
using namespace BT;

class EmergencyBrake: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient brakeService;
  ros::Publisher logPub;
  std_msgs::String logInfo;
  float currentRate;
public:
  EmergencyBrake(const std::string& name, const BT::NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub);
  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {return {};}
  void halt() override ;
};
