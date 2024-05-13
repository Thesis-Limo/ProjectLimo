#include "behaviortree_cpp_v3/action_node.h"
#include "position.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace BT;

class CreatePath: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
  ros::Publisher logPub;
  std_msgs::String logInfo;

public:
    CreatePath(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
    void halt() override;
};