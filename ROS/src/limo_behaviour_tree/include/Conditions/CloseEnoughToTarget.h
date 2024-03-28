#include "behaviortree_cpp_v3/condition_node.h"
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include "position.hpp"
#include "nav_msgs/Odometry.h"
using namespace BT;

class CloseEnoughToTarget: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subTarget;
    ros::Subscriber subPosition;
    Point3D currentPos;
    Point3D targetPos;
    float distanceToClose;

    void CallBackTarget(const geometry_msgs::Point::ConstPtr& msg);
    void CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg);
public:
    CloseEnoughToTarget(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
};