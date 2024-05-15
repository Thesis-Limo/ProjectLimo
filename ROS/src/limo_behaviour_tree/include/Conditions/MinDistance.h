#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include "Node.h"

using namespace BehaviourTree;
class MinDistance: public Node
{
private:
    ros::Subscriber sub;
    float minDistance;
    float currentDistance = __FLT_MAX__;
    void CallBackPoints(const sensor_msgs::LaserScan::ConstPtr& msg);
public:
    MinDistance(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};