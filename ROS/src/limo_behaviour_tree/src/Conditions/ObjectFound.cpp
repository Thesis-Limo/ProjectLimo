#include "ObjectFound.h"
#include <std_srvs/Trigger.h>

ObjectFound::ObjectFound(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{
    logInfo.data = "Target is not found";
}

void ObjectFound::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
    nh = nodehandle;
    this->logPub = logPub;
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_target", 100);
}

BT::NodeStatus ObjectFound::tick()
/*
 * checks the target if it exist
*/
{   
    std_srvs::Trigger msg;
    if(this->pathService.call(msg))
    {
        if(msg.response.success)
        {
            this->logPub.publish(logInfo);
            return NodeStatus::SUCCESS;
        }
    }
    return NodeStatus::FAILURE;
}