#include "CreatePath.h"
#include "limo_behaviour_tree/PathType.h"
CreatePath::CreatePath(const std::string& name, const NodeConfiguration& conf)
    :ActionNodeBase(name, conf)
{
  logInfo.data = "Have found the object, now create the path";
}
void CreatePath::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
    nh = nodehandle;
    client = nh.serviceClient<limo_behaviour_tree::PathType>("/BT/create_path");
    this->logPub = logPub;
}
NodeStatus CreatePath::tick()
/*
 * Gets information form the previous condition and calls function /GoalPos, so that it can calculate the trajectory
*/
{
    limo_behaviour_tree::PathType msg;
    msg.request.pathType = 0;
    if(client.call(msg))
    {
        //LogInfo
        this->logPub.publish(logInfo);
        return NodeStatus::SUCCESS;    
    }
    return NodeStatus::FAILURE;
}
void CreatePath::halt(){
  }