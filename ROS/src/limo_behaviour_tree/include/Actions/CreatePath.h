#include "behaviortree_cpp_v3/action_node.h"
#include "position.hpp"
#include <ros/ros.h>
using namespace BT;

class CreatePath: public ActionNodeBase
{
private:
  ros::NodeHandle nh;
  ros::ServiceClient client;

public:
    CreatePath(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {InputPort<Point3D>("target", "the target")};}
    void halt() override;
};