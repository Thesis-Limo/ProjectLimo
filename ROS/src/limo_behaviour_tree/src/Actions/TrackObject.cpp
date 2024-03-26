#include "TrackObject.h"
#include "limo_behaviour_tree/TypeObjectTracking.h"
TrackObject::TrackObject(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf), nh("")
{
    client = nh.serviceClient<limo_behaviour_tree::TypeObjectTracking>("/BT/FindObject");
}
BT::NodeStatus TrackObject::tick()
{
  limo_behaviour_tree::TypeObjectTracking srv;
  srv.request.objectID = 0;
  std::cout << "tracking" << std::endl;
  Position3D point = {1,1,1};
  setOutput("goal",point);
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