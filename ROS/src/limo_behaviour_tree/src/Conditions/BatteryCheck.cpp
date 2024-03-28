#include "BatteryCheck.h"
BatteryCheck::BatteryCheck(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{}

void BatteryCheck::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    sub = nh.subscribe("/limo_status", 1000,&BatteryCheck::BatteryCallBack, this);
}

NodeStatus BatteryCheck::tick()
{   
    if(BatteryLevel < 0)
        return NodeStatus::RUNNING;
    else if (BatteryLevel < 10)
        return NodeStatus::FAILURE;
    return NodeStatus::SUCCESS;
}
void BatteryCheck::BatteryCallBack(const limo_base::LimoStatus& msgs)
{
    BatteryLevel = (msgs.battery_voltage/ maxBatteryVoltage) * 100;
}