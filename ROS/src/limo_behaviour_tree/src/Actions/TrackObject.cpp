#include "TrackObject.h"
#include "limo_behaviour_tree/TypeObjectTracking.h"
#include "geometry_msgs/Point.h"
TrackObject::TrackObject(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
    :Node(nodehandle, logPub)
{
  currentTargetID = nh.param<int>("currentTargetID",39);
  client = nh.serviceClient<limo_behaviour_tree::TypeObjectTracking>("/TrackID");
}

NodeStatus TrackObject::Tick()
/*
 * calls service for tracking with name /BT/FindObject
*/
{
  limo_behaviour_tree::TypeObjectTracking srv;
  srv.request.objectID = currentTargetID;
  if(client.call(srv))
  {
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}
