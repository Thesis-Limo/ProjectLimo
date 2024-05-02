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
#include "image_geometry/pinhole_camera_model.h"
#include "limo_yolo/transformedLidarPoints.h"

#include <chrono>
#include <queue>
#include "position.hpp"
using namespace sensor_msgs;
using namespace cv_bridge;

struct DataFrame
{
    uint32_t seq;
    limo_yolo::transformedLidarPoints lidar;
    Image currentImage;
};


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
    int objectId;
    image_geometry::PinholeCameraModel cam_model_;

    std::queue<DataFrame> pushedFrames;
    DataFrame current;
    int id;

    std::string baseLinkFrame;
    std::string laserFrame;
    std::string cameraFrame;
    //tf::StampedTransform transform;
    
    std::vector<Point3D> ConvertToLidar(const LaserScan& laser);

public:
    YoloProjection(const ros::NodeHandle& nodehandle);
    void CallbackImageAndLidar(const Image& image, const limo_yolo::transformedLidarPoints& laser);
    void CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes);
    bool SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res);
};