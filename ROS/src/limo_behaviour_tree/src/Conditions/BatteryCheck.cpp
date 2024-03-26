#include "BatteryCheck.h"
BatteryCheck::BatteryCheck(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf), nh("")
{
    sub = nh.subscribe("/limo_status", 1000,&BatteryCheck::BatteryCallBack, this);
}
BT::NodeStatus BatteryCheck::tick()
{   
    if(BatteryLevel < 0)
        return BT::NodeStatus::RUNNING;
    else if (BatteryLevel < 100)
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}
void BatteryCheck::BatteryCallBack(const limo_base::LimoStatus& msgs)
{
    BatteryLevel = (msgs.battery_voltage/ maxBatteryVoltage) * 100;
}