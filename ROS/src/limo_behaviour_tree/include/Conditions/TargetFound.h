#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace BT;

class TargetFound: public ConditionNode
{
private:
    ros::NodeHandle nh; 
    ros::Subscriber subTarget;
    //void Callback();
public:
    TargetFound(const std::string& name, const NodeConfiguration& conf);
    NodeStatus tick() override;
    static PortsList providedPorts(){return {};}
};