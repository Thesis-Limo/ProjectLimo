#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"
#include <limo_motion_controller/OverrideMotion.h>
#include <std_msgs/String.h>
using namespace BT;
class MoveBack: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient moveBackService;
  ros::Publisher logPub;
  std_msgs::String logInfo;

  limo_motion_controller::OverrideMotion moveMsg;

  float durationGoingBack;
  float speedGoingBack;
public:
  MoveBack(const std::string& name, const BT::NodeConfiguration& conf);
  void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);

  BT::NodeStatus tick() override;
  static auto providedPorts() -> PortsList {
    return {BT::InputPort<std::string>("message")};
  }
  void halt() override ;
};
