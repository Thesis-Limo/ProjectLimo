#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <position.hpp>
#include "nav_msgs/Odometry.h"
using namespace BT;

class ToCloseToTarget: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subTarget;
    ros::Subscriber subPosition;
    Point3D currentPos;
    Point3D targetPos;
    float distanceToDecideToClose;

    void CallBackTarget(const geometry_msgs::Point::ConstPtr& msg);
    void CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg);
public:
    ToCloseToTarget(const std::string& name, const NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);
    NodeStatus tick() override;
    
    static PortsList providedPorts(){return {};}
};