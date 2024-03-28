#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace BT;
class MinDistance: public ConditionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    float minDistance;
    float currentDistance = __FLT_MAX__;
    void CallBackPoints(const PointCloud::ConstPtr& msg);
public:
    MinDistance(const std::string& name, const BT::NodeConfiguration& conf);
    void Initialize(const ros::NodeHandle& nodehandle);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){return {};}
};