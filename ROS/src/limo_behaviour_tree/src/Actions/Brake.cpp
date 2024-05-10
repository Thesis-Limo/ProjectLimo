#include "Brake.h"

Brake::Brake(const std::string& name, const NodeConfiguration& conf)
    :ActionNodeBase(name, conf)
{
    this->logInfo.data = "Something is in front I need to brake right now";
    this->brakeMsg.request.speed = 0;
    this->brakeMsg.request.angle = __FLT_MAX__;
    this->brakeMsg.request.duration = durationSlowDown;
    this->brakeMsg.request.id = "Brake";
}
void Brake::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
    this->nh = nodehandle;
    this->brakeService = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");
    this->logPub = logPub;
}
NodeStatus Brake::tick()
/*
 * calls service for tracking with name /BT/Brake to execute braking
*/
{
    if(this->brakeService.call(brakeMsg))
    { 
        ROS_INFO("Brake");
        this->logPub.publish(logInfo);
        return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;

}
void Brake::halt(){
  }