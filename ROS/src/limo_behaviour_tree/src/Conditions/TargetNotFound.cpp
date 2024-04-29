#include "TargetNotFound.h"
TargetNotFound::TargetNotFound(const std::string& name, const NodeConfiguration& conf)
    :ConditionNode(name, conf), nh("")
{
    //sub = nh.subscriber<>(,100,&TargetFound::Callback, this);
}
NodeStatus TargetNotFound::tick()
{
    return NodeStatus::FAILURE;
}
// void TargetFound::CallBack()
// {

// }