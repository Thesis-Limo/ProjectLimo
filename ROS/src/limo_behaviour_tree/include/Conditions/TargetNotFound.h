#include <geometry_msgs/Twist.h>
#include "Node.h"
using namespace BehaviourTree;

class TargetNotFound: public Node
{
private:
    ros::ServiceClient pathService;
public:
    TargetNotFound(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};