#pragma once
#include "position.hpp"
#include <Node.h>
using namespace BehaviourTree;

class CreatePath: public Node
{
private:
  ros::ServiceClient client;

public:
    CreatePath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};