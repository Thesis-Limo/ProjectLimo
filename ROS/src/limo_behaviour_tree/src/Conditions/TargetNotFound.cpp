#include "TargetNotFound.h"
#include <std_srvs/Trigger.h>


TargetNotFound::TargetNotFound(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub,"Target is not found")
{
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_target", 100);
    ros::service::waitForService("/check_target", 10000); 
}

NodeStatus TargetNotFound::Tick()
{
    std_srvs::Trigger msg;

    if (this->pathService.exists())
        ROS_INFO("it exists");
    ROS_INFO("fff");
    
    if(this->pathService.call(msg))
    {
        if(msg.response.success)
        {
            Log();
            return NodeStatus::FAILURE;
        }
    }
    return NodeStatus::SUCCESS;
}