#include "ObjectFound.h"
ObjectFound::ObjectFound(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{}

void ObjectFound::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    sub = nh.subscribe("/ObjectPos", 100,&ObjectFound::ObjectFoundCallBack, this);
}
void ObjectFound::ObjectFoundCallBack(const geometry_msgs::Point& msgs)
/*
 * if target is found set the target else set target to false
*/
{
    if(msgs.x != __FLT_MAX__ && msgs.y != __FLT_MAX__ && msgs.z != __FLT_MAX__)
    {
        found = true;
        targetpos = {msgs.x, msgs.y, msgs.z};
    }
    else 
        found = false;
}

BT::NodeStatus ObjectFound::tick()
/*
 * pushes position of target if the goal has been found
*/
{   
    //send pos to next part
    if(!found)
        return NodeStatus::FAILURE;
    return NodeStatus::SUCCESS;
}