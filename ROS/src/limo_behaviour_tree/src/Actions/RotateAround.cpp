#include "RotateAround.h"
#include <geometry_msgs/Twist.h>
#include <limo_behaviour_tree/PathType.h>
RotateAround::RotateAround(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{
  logInfo.data = "Object isn't found so I need turn around looking for the object";
}

void RotateAround::Initialize(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
{
  nh = nodehandle;
  client = nh.serviceClient<limo_behaviour_tree::PathType>("/BT/create_path");
  this->logPub = logPub;
}

BT::NodeStatus RotateAround::tick()
/*
 * This function does at the moment nothing
*/
{
  limo_behaviour_tree::PathType msg;
  msg.request.pathType = 1;
  ROS_INFO("Rotate");

  if(client.call(msg))
  {
    //LogInfo
    this->logPub.publish(logInfo);
    return NodeStatus::SUCCESS;    
  }
  return NodeStatus::FAILURE;
  
}
void RotateAround::halt(){
  }