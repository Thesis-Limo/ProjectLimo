#include "TargetFound.h"
TargetFound::TargetFound(const std::string& name, const NodeConfiguration& conf)
    :ConditionNode(name, conf), nh("")
{
    //sub = nh.subscriber<>(,100,&TargetFound::Callback, this);
}
NodeStatus TargetFound::tick()
{
    return NodeStatus::SUCCESS;
}
// void TargetFound::CallBack()
// {

// }