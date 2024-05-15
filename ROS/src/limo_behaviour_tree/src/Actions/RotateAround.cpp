#include "RotateAround.h"
#include <geometry_msgs/Twist.h>
#include <limo_behaviour_tree/PathType.h>

RotateAround::RotateAround(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub,  "Object isn't found so I need turn around looking for the object")
{
  client = nh.serviceClient<limo_behaviour_tree::PathType>("/BT/create_path");
}
NodeStatus RotateAround::Tick()
/*
 * This function does at the moment nothing
*/
{
  limo_behaviour_tree::PathType msg;
  msg.request.pathType = 1;

  if(client.call(msg))
  {
    //LogInfo
    Log();
    return NodeStatus::SUCCESS;
  }
   //TODO this is temp normally remove the Log and return failure
  Log();

  return NodeStatus::SUCCESS;
}