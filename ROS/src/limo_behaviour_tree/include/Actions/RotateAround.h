#include "Node.h"

using namespace BehaviourTree;
class RotateAround: public Node
{
private:
  ros::ServiceClient client;
public:
  RotateAround(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
