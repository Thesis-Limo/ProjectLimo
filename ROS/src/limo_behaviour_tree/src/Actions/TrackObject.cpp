#include "TrackObject.h"
TrackObject::TrackObject(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf)
{}
BT::NodeStatus TrackObject::tick()
{
    return BT::NodeStatus::SUCCESS;
}
void TrackObject::halt(){
  }