#include "CheckPath.h"
#include <std_srvs/Trigger.h>

CheckPath::CheckPath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub,"There is not a path to my goal")
{
    //TODO if service is ready we can use i    ros::service::waitForService("/check_path", 10000); 
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_path");
}

NodeStatus CheckPath::Tick()
/*
 * Checks if there is already a plath or not returns success if no math is found
*/
{   
    std_srvs::Trigger msg;
    if(this->pathService.call(msg))
    {
        if(!msg.response.success)
        {
            Log();
            return NodeStatus::FAILURE;
        }
    }
    return NodeStatus::SUCCESS;
}
