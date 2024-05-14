#include "TrackObject.h"
#include "limo_behaviour_tree/TypeObjectTracking.h"
#include "geometry_msgs/Point.h"

TrackObject::TrackObject(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)
{
  client = nh.serviceClient<limo_behaviour_tree::TypeObjectTracking>("/BT/FindObject");
}

NodeStatus TrackObject::Tick()
/*
 * calls service for tracking with name /BT/FindObject
*/
{
  limo_behaviour_tree::TypeObjectTracking srv;
  srv.request.objectID = 0;
  if(client.call(srv))
  {
    if(srv.response.position.x >= __FLT_MAX__)
      return NodeStatus::RUNNING;
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}
