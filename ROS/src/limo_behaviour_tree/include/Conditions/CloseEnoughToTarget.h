#include <geometry_msgs/PoseArray.h>
#include "position.hpp"
#include "nav_msgs/Odometry.h"
#include "Node.h"
using namespace BehaviourTree;

class CloseEnoughToTarget: public Node
{
private:
    ros::Subscriber subTarget;
    ros::Subscriber subPosition;
    Point3D currentPos;
    Point3D targetPos;
    float distanceToClose;

    void CallBackTarget(const geometry_msgs::Point::ConstPtr& msg);
    void CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg);
public:
    CloseEnoughToTarget(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};