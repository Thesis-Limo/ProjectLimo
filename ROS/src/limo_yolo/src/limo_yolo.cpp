#include <ros/ros.h>
#include <ros/package.h>
//#include "YoloProjection.h"



#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using message_filters::TimeSynchronizer;
using namespace sensor_msgs;
void sync_cb(const Image::ConstPtr &img_msg, const Image::ConstPtr &md_msg)
{
    ROS_INFO("tesss");

}
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_yolo");
    ros::NodeHandle nh;
    //YoloProjection pro(nh);
    //ros::Rate r(10); 
    message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<Image> metadata_sub(nh, "/scan", 1);
    TimeSynchronizer<Image, Image> sync(image_sub, metadata_sub, 10);
    sync.registerCallback(boost::bind(&sync_cb, _1, _2));
    ros::spin();

    return 0;
}