#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
#include "Node.h"

using namespace BehaviourTree;
class MinDistance: public Node
{
private:
    ros::Subscriber sub;
    ros::Subscriber subOdom;
    float minDistance;
    float currentDistance = __FLT_MAX__;
    
    float distanceToClose;
    ros::Time prevTimestamp;
    geometry_msgs::Point prevPos;
    float speedSqr = 0;
    void CallBackPoints(const sensor_msgs::LaserScan::ConstPtr& msg);
    void CallBackPosition(const nav_msgs::Odometry::ConstPtr& msg);

public:
    MinDistance(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};