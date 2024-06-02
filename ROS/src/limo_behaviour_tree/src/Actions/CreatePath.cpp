
#include "CreatePath.h"
#include "limo_behaviour_tree/PathType.h"
CreatePath::CreatePath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
    :Node(nodehandle, logPub,  "Have found the object, now create the path")
{
    client = nh.serviceClient<limo_behaviour_tree::PathType>("/BT/create_path");
    this->controllerClient = nh.serviceClient<limo_motion_controller::OverrideMotion>("/override_plan");

}
NodeStatus CreatePath::Tick()
/*
 * Gets information form the previous condition and calls function /GoalPos, so that it can calculate the trajectory
*/
{
    limo_motion_controller::OverrideMotion mesg;
    mesg.request.speed = 0;
    mesg.request.angle = __FLT_MAX__;
    mesg.request.duration = 0;
    client.call(mesg);
    limo_behaviour_tree::PathType msg;
    msg.request.pathType = 0;
    if(client.call(msg))
    {
        //LogInfo
        Log();
        return NodeStatus::SUCCESS;    
    }
    return NodeStatus::FAILURE;
}