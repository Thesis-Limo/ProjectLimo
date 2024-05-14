#pragma once

#include <geometry_msgs/Twist.h>
#include <limo_motion_controller/OverrideMotion.h>
#include "Node.h"
using namespace BehaviourTree;

class EmergencyBrake: public Node
{
private:
  ros::ServiceClient brakeService;
  float currentRate;
public:
  EmergencyBrake(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
