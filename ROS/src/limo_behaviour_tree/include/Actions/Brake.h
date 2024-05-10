#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <limo_motion_controller/MovementController.h>
#include <limo_motion_controller/OverrideMotion.h>

using namespace BT;

class Brake: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient brakeService;
  ros::Publisher logPub;
  std_msgs::String logInfo;
  limo_motion_controller::OverrideMotion brakeMsg;
  float durationSlowDown = 1;
public:
  Brake(const std::string& name, const BT::NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {};
  }
  void halt() override ;
};
