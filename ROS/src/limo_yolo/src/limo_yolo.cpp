#include <ros/ros.h>
#include <ros/package.h>
#include "YoloProjection.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include "limo_yolo/transformedLidarPoints.h"

YoloProjection* pro = nullptr;
void callback(const sensor_msgs::Image::ConstPtr& img, const limo_yolo::transformedLidarPoints::ConstPtr& laser){
    pro->CallbackImageAndLidar(*img, *laser);
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_yolo");
    ros::NodeHandle nh("");
    pro = new YoloProjection(nh);
    ros::Rate r(10); 
    message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<limo_yolo::transformedLidarPoints> info_sub(nh, "/scan", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, limo_yolo::transformedLidarPoints> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}