#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <limo_base/LimoStatus.h>
using namespace BT;

class BatteryCheck: public ConditionNode
{
private:
    int BatteryLevel = -1;
    float maxBatteryVoltage = 12.6;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    
public:
    void BatteryCallBack(const limo_base::LimoStatus& msgs);
    BatteryCheck(const std::string& name, const BT::NodeConfiguration& conf);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
};