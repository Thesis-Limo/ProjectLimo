#include "BatteryCheck.h"
BatteryCheck::BatteryCheck(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{}

void BatteryCheck::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    sub = nh.subscribe("/limo_status", 1000,&BatteryCheck::BatteryCallBack, this);
    maxBatteryVoltage = nh.param<float>("minBatteryVoltage",12.6);
    batteryToLow = nh.param<float>("batteryToLow",10);
}

NodeStatus BatteryCheck::tick()
{   
    std::cout << BatteryLevel <<  "--batteryOK\n";

    if(BatteryLevel < 0)
        return NodeStatus::RUNNING;
    else if (BatteryLevel < batteryToLow)
        return NodeStatus::FAILURE;
    return NodeStatus::SUCCESS;
}
void BatteryCheck::BatteryCallBack(const limo_base::LimoStatus& msgs)
{
    BatteryLevel = (msgs.battery_voltage/ maxBatteryVoltage) * 100;
}