#include "MinDistance.h"
#include <boost/foreach.hpp>

MinDistance::MinDistance(const ros::NodeHandle& nodehandle,const ros::Publisher& logPub)
  :Node(nodehandle, logPub)

{
    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1, &MinDistance::CallBackPoints, this);
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
void MinDistance::CallBackPoints(const sensor_msgs::LaserScan::ConstPtr&  laser)
/*
 * checks every pixel from the depth camera and looks for closest distance
*/
{ 
    float min = 0.61;
    float minDistance = __FLT_MAX__;

    float min_dist = laser->range_min;
    float max_dist = laser->range_max;
    int startId = 0;
    for (int i = 0; i < laser->ranges.size(); i++)
    {
        float distance = laser->ranges[i];
        if(min_dist < distance && max_dist > distance)
        {
            float angle = laser->angle_min + i * laser->angle_increment;
            if (abs(angle) > min) continue;
            if(minDistance > distance) minDistance = distance;
        }
    }
    currentDistance = minDistance;
}