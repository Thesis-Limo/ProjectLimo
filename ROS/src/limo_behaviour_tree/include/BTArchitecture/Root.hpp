#include "Node.h"
#include "BatteryCheckSub.hpp"
#include "MovingRobot.hpp"

using namespace BehaviourTree;

class Root: public Node
{
private:
    BatteryCheckSub batteryCheckSub;
    MovingRobot movingRobot;
public:
    Root(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

Root::Root(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), batteryCheckSub(nodehandle, logPub), movingRobot(nodehandle, logPub)
{
}
NodeStatus Root::Tick()
{
    ROS_INFO("ROOT");
    NodeStatus x = batteryCheckSub.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    return movingRobot.Tick();   
}
