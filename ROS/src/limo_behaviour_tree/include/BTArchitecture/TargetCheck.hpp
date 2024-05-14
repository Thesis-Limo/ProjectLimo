#include "Node.h"
#include "TargetNotFound.h"
#include "RotateAround.h"
#include "CheckPath.h"

using namespace BehaviourTree;

class TargetCheck: public Node
{
private:
    TargetNotFound targetNotFound;
    RotateAround rotateAround;
    CheckPath checkPath;
public:
    TargetCheck(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

TargetCheck::TargetCheck(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), targetNotFound(nodehandle, logPub), rotateAround(nodehandle, logPub), checkPath(nodehandle, logPub)
{
}
NodeStatus TargetCheck::Tick()
{
    NodeStatus x = checkPath.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    x = targetNotFound.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    return rotateAround.Tick();
}
