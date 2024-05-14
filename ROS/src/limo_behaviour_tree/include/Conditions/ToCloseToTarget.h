#include <position.hpp>
#include "nav_msgs/Odometry.h"
#include "Node.h"

using namespace BehaviourTree;
class ToCloseToTarget: public Node
{
private:
    ros::Subscriber subTarget;
    ros::Subscriber subPosition;
    Point3D currentPos;
    Point3D targetPos;
    float distanceToDecideToClose;

    void CallBackTarget(const geometry_msgs::Point::ConstPtr& msg);
    void CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg);
public:
    ToCloseToTarget(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};