#include "Node.h"
#include "MoveBack.h"
using namespace BehaviourTree;

class DistanceBased: public Node
{
private:
    MoveBack moveback;
public:
    DistanceBased(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

DistanceBased::DistanceBased(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
  :Node(nodehandle, logPub), moveback(nodehandle, logPub)
{
}
NodeStatus DistanceBased::Tick()
{
    ROS_INFO("Distance based");

    return moveback.Tick();
}