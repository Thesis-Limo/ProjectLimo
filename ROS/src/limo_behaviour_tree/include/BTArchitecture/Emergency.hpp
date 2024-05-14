#include "Node.h"
#include "MinDistance.h"
#include "DistanceBased.hpp"

using namespace BehaviourTree;

class Emergency: public Node
{
private:
    DistanceBased distanceBased;
    MinDistance minDistance;
public:
    Emergency(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

Emergency::Emergency(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), distanceBased(nodehandle, logPub), minDistance(nodehandle, logPub)
{
}
NodeStatus Emergency::Tick()
{
    NodeStatus x = minDistance.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    return distanceBased.Tick();   
}
