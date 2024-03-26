#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace BT;

class SpeedZero: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    geometry_msgs::Point prevPos;
    ros::Time prevTimestamp;
    float speedSqr = 0;
    void CallBackOdom(const nav_msgs::Odometry::ConstPtr& msg);
public:
    SpeedZero(const std::string& name, const NodeConfiguration& conf);
    NodeStatus tick() override;
    static PortsList providedPorts(){return {};}
};