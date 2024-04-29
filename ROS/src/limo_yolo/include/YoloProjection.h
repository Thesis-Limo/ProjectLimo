#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <limo_behaviour_tree/TypeObjectTracking.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace cv_bridge;
class YoloProjection
{
private:
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listenerTransform;
    ros::Publisher yoloImagePub;
    ros::Publisher posObjectPub;
    ros::ServiceServer targetService;
    ros::Subscriber sub;
    CameraInfo cameraInfo;
    cv::Mat K;
    int objectId;
    bool gotInfoBackFromYolo = false;

    std::vector<uint8_t> currentImage;
    LaserScan currentLidarScan;

public:
    YoloProjection(const ros::NodeHandle& nodehandle);
    void CallbackImageAndLidar(const Image& image, const LaserScan& laser);
    void CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes);
    bool SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res);
};