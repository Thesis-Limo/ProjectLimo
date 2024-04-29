#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include "position.hpp"

using namespace BT;

class CheckPath: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    
public:
    // void CheckPathCallBack(const limo_base::LimoStatus& msgs);
    CheckPath(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
    
};