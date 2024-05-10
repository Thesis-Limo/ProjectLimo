#include "CheckPath.h"
#include <std_srvs/Trigger.h>
CheckPath::CheckPath(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{
  logInfo.data = "There is not a path to my goal";
}

void CheckPath::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
    nh = nodehandle;
    this->logPub = logPub;
    this->pathService = nh.serviceClient<std_srvs::Trigger>("/check_path");
}

NodeStatus CheckPath::tick()
/*
 * Checks if there is already a plath or not returns success if no math is found
*/
{   
    std_srvs::Trigger msg;
    if(this->pathService.call(msg))
    {
        if(msg.response.success)
        {
            this->logPub.publish(logInfo);
            return NodeStatus::FAILURE;
        }
    }
    ROS_INFO("path not found");
    return NodeStatus::SUCCESS;
}
