#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Point.h>
#include "Node.h"
using namespace BehaviourTree;

class SpeedNotZero: public Node
{
private:
    ros::Subscriber sub;
    geometry_msgs::Point prevPos;
    ros::Time prevTimestamp;
    float speedSqr = 0;
    void CallBackOdom(const nav_msgs::Odometry::ConstPtr& msg);
public:
    SpeedNotZero(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};