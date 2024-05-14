#include "MinDistance.h"
#include <boost/foreach.hpp>

MinDistance::MinDistance(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)

{
    sub = nh.subscribe<PointCloud>("/camera/depth/points",1, &MinDistance::CallBackPoints, this);
    minDistance = nh.param<float>("minDistanceWhenBraking",0.25f);
}
NodeStatus MinDistance::Tick()
/*
 * if the current distance is to close to the object then return success
*/
{
    if(currentDistance < minDistance)
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}
void MinDistance::CallBackPoints(const PointCloud::ConstPtr&  msg)
/*
 * checks every pixel from the depth camera and looks for closest distance
*/
{  
    float minDistance = __FLT_MAX__;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        if(minDistance > pt.z) minDistance = pt.z;
    }
    currentDistance = minDistance;
}