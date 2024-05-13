#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "position.hpp"

using namespace BT;

class CheckPath: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher logPub;
    std_msgs::String logInfo;
    ros::ServiceClient pathService;
    
public:
    CheckPath(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
    
};