#include "Node.h"
#include "MoveToTarget.h"
#include "TargetCheck.hpp"
#include "Emergency.hpp"

using namespace BehaviourTree;

class MovingRobot: public Node
{
private:
    Emergency emergency;
    TargetCheck targetCheck;
    MoveToTarget moveToTarget;
public:
    MovingRobot(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

MovingRobot::MovingRobot(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), emergency(nodehandle, logPub), targetCheck(nodehandle, logPub),moveToTarget(nodehandle, logPub)
{
}
NodeStatus MovingRobot::Tick()
{
    ROS_INFO("MOVING ROBOT");

    NodeStatus x = emergency.Tick();
    if((x == NodeStatus::SUCCESS) || (x == NodeStatus::RUNNING)) return x; 
    x = targetCheck.Tick();
    if((x == NodeStatus::SUCCESS) || (x == NodeStatus::RUNNING)) return x; 
    return moveToTarget.Tick();   
}
