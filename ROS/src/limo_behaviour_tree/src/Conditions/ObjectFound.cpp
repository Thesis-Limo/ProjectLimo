#include "ObjectFound.h"
#include <std_srvs/Trigger.h>

ObjectFound::ObjectFound(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub,"Target is not found")
{
    ros::service::waitForService("/check_target", 10000); 
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_target", 100);
}

NodeStatus ObjectFound::Tick()
/*
 * checks the target if it exist
*/
{   
    std_srvs::Trigger msg;
    if(this->pathService.call(msg))
    {
        if(!msg.response.success)
        {
            Log();
            return NodeStatus::SUCCESS;
        }
    }
    return NodeStatus::FAILURE;
}