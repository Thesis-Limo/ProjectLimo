#include <ros/ros.h>
#include <ros/package.h>
#include "YoloProjection.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

YoloProjection* pro = nullptr;
void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::LaserScan::ConstPtr& laser){
ROS_INFO("ree");
    pro->CallbackImageAndLidar(*img, *laser);
    
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_yolo");
    ros::NodeHandle nh("~");
    pro = new YoloProjection(nh);
    ros::Rate r(10); 
    std::string scanTopic = nh.param<std::string>("scan", "/scan");
    std::string cameraImage = nh.param<std::string>("cameraImage", "/camera/rgb/image_raw");

    message_filters::Subscriber<Image> image_sub(nh, cameraImage, 1);
    message_filters::Subscriber<LaserScan> info_sub(nh,scanTopic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}