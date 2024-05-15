#include "TargetNotFound.h"
#include <std_srvs/Trigger.h>


TargetNotFound::TargetNotFound(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
    ros::service::waitForService("/check_target", 10000); 
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_target", 100);
}

NodeStatus TargetNotFound::Tick()
{
    std_srvs::Trigger msg;
    
    if(this->pathService.call(msg))
    {
        if(msg.response.success)
        {
            return NodeStatus::FAILURE;
        }
    }
    return NodeStatus::SUCCESS;
}