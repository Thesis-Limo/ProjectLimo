
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
bool once = false;
void CallBackPoints(const PointCloud::ConstPtr&  msg)
{
    if(once) return;
    std::ofstream myfile("test.csv");
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        myfile << pt.x << ";" << pt.y << ";" << pt.z << "\n";
    }
    myfile.close();
    once = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"GroundIdentification");
    ROS_INFO("test");
    ros::NodeHandle nh("");
    auto sub = nh.subscribe<PointCloud>("/camera/depth/points",100, &CallBackPoints);
    ros::spin();
    return 0;
}