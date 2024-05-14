#include "Node.h"
#include "BatteryCheck.h"
#include "Brake.h"

using namespace BehaviourTree;

class BatteryCheckSub: public Node
{
private:
    BatteryCheck batteryCheck;
    Brake brake;
public:
    BatteryCheckSub(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

BatteryCheckSub::BatteryCheckSub(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), batteryCheck(nodehandle, logPub), brake(nodehandle, logPub)
{
}
NodeStatus BatteryCheckSub::Tick()
{
    NodeStatus x = batteryCheck.Tick();
    if((x == NodeStatus::SUCCESS) || (x == NodeStatus::RUNNING)) return x; 
    ROS_INFO("BATTERYCHECK");
    return brake.Tick();   
}
