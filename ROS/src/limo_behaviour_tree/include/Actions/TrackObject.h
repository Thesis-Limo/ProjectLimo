#include "Node.h"
using namespace BehaviourTree;

class TrackObject: public Node
{
private:
  ros::ServiceClient client;
  int currentTargetID = 0;
public:
  TrackObject(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
