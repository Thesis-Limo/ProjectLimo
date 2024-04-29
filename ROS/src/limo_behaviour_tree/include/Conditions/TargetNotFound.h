#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace BT;

class TargetNotFound: public ConditionNode
{
private:
    ros::NodeHandle nh; 
    ros::Subscriber subTarget;
    //void Callback();
public:
    TargetNotFound(const std::string& name, const NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);
    NodeStatus tick() override;
    static PortsList providedPorts(){return {};}
};