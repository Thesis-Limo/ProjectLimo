#include "TrackObject.h"
#include "limo_behaviour_tree/TypeObjectTracking.h"
#include "geometry_msgs/Point.h"
TrackObject::TrackObject(const std::string& name, const NodeConfiguration& conf)
: ActionNodeBase(name, conf)
{
}
void TrackObject::Initialize(const ros::NodeHandle& nodehandle)
{
  nh = nodehandle;
  currentTargetID = nh.param<int>("currentTargetID",39);
  client = nh.serviceClient<limo_behaviour_tree::TypeObjectTracking>("/TrackID");

}

BT::NodeStatus TrackObject::tick()
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
void TrackObject::halt(){
}
