#include "CheckPath.h"
CheckPath::CheckPath(const std::string& name, const BT::NodeConfiguration& conf)
    :BT::ConditionNode(name, conf)
{}

void CheckPath::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    //TODO Determine how it is going to look like the path
    //sub = nh.subscribe("/path", 1000,&CheckPath::CheckPathCallBack, this);
}

NodeStatus CheckPath::tick()
/*
 * TODO needs to be implemented
*/
{   
    return NodeStatus::SUCCESS;
}
// void CheckPath::CheckPathCallBack(const & msgs)
// {

// }