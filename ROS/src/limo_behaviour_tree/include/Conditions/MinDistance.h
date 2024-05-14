#include <pcl_ros/point_cloud.h>
#include "Node.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace BehaviourTree;
class MinDistance: public Node
{
private:
    ros::Subscriber sub;
    float minDistance;
    float currentDistance = __FLT_MAX__;
    void CallBackPoints(const PointCloud::ConstPtr& msg);
public:
    MinDistance(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub);
    NodeStatus Tick() override;
};