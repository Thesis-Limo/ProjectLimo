#include "TargetNotFound.h"
#include <std_srvs/Trigger.h>
TargetNotFound::TargetNotFound(const std::string& name, const NodeConfiguration& conf)
    :ConditionNode(name, conf)
{
    logInfo.data = "Target is not found";
}

void TargetNotFound::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
    nh = nodehandle;
    this->logPub = logPub;
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_target", 100);
}

NodeStatus TargetNotFound::tick()
{
    ROS_INFO("targetnotfound");
    std_srvs::Trigger msg;
    if(this->pathService.call(msg))
    {
        if(msg.response.success)
        {
            this->logPub.publish(logInfo);
            return NodeStatus::FAILURE;
        }
    }
    return NodeStatus::SUCCESS;
}