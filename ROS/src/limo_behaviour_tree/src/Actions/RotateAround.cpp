#include "RotateAround.h"
#include <geometry_msgs/Twist.h>
#include <limo_behaviour_tree/PathType.h>
RotateAround::RotateAround(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}

void RotateAround::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    client = nh.serviceClient<limo_behaviour_tree::PathType>("/BT/ChangePathType");
}

BT::NodeStatus RotateAround::tick()
/*
 * This function does at the moment nothing
*/
{
  limo_behaviour_tree::PathType msg;
  msg.request.pathType = 1;
  if(client.call(msg))
    return NodeStatus::SUCCESS;   
  return NodeStatus::FAILURE;
}
void RotateAround::halt(){
  }