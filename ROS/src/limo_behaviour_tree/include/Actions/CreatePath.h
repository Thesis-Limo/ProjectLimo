#pragma once
#include <limo_motion_controller/OverrideMotion.h>
#include "position.hpp"
#include <geometry_msgs/Twist.h>
#include <Node.h>
using namespace BehaviourTree;

class CreatePath: public Node
{
private:
  ros::ServiceClient client;
  ros::ServiceClient controllerClient;

public:
    CreatePath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};