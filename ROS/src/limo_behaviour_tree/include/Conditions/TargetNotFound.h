#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

using namespace BT;

class TargetNotFound: public ConditionNode
{
private:
    ros::NodeHandle nh; 
    ros::Publisher logPub;
    std_msgs::String logInfo;
    ros::ServiceClient pathService;
    
public:
    TargetNotFound(const std::string& name, const NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus tick() override;
    static PortsList providedPorts(){return {};}
};