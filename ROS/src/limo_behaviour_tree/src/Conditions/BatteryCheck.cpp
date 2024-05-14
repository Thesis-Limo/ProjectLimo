#include "BatteryCheck.h"
BatteryCheck::BatteryCheck(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
    :Node(nodehandle, logPub)
{
    sub = nh.subscribe("/limo_status", 1000,&BatteryCheck::BatteryCallBack, this);
    maxBatteryVoltage = nh.param<float>("minBatteryVoltage",12.6);
    batteryToLow = nh.param<float>("batteryToLow",10);
}

NodeStatus BatteryCheck::Tick()
/*
 * creturns failure if the battery is below a certain percentage
*/
{   
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