

#include "Brake.h"

Brake::Brake(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
    :Node(nodehandle, logPub)
{
    this->brakeService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
    this->brakeMsg.request.speed = 0;
    this->brakeMsg.request.angle = __FLT_MAX__;
    this->brakeMsg.request.duration = durationSlowDown;
}
NodeStatus Brake::Tick()
/*
 * calls service for tracking with name /BT/Brake to execute braking
*/
{
    if(this->brakeService.call(brakeMsg))
    { 
        ROS_INFO("Brake");
        Log();
        return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
}
