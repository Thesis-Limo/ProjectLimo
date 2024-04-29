#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include "position.hpp"

using namespace BT;

class ObjectFound: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    Point3D targetpos;
    bool found = false;

public:
    ObjectFound(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);
    void ObjectFoundCallBack(const geometry_msgs::Point& msgs);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {
        OutputPort<Point3D>("goal")
        };}
};