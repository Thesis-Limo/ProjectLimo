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
  client = nh.serviceClient<limo_behaviour_tree::TypeObjectTracking>("/BT/FindObject");
}

BT::NodeStatus TrackObject::tick()
/*
 * calls service for tracking with name /BT/FindObject
*/
{
  limo_behaviour_tree::TypeObjectTracking srv;
  srv.request.objectID = 0;
  std::cout << "tracking" << std::endl;
  if(client.call(srv))
  {
    if(srv.response.position.x >= __FLT_MAX__)
      return NodeStatus::RUNNING;
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::FAILURE;
}
void TrackObject::halt(){
}
