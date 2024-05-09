#include <ros/ros.h>
#include <queue>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <limo_behaviour_tree/TypeObjectTracking.h>
#include <limo_yolo/map.h>

#include "position.hpp"

using namespace sensor_msgs;

struct DataFrame
{
    uint32_t seq;
    LaserScan lidar;
    Image currentImage;
    geometry_msgs::Pose currentPose;
};

class YoloProjection
{
private:
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listenerTransform;
    ros::Publisher yoloImagePub;
    ros::Publisher mapPub;
    ros::Publisher mapTempPub;
    ros::ServiceServer targetService;
    ros::Subscriber sub;
    ros::Subscriber subOdom;
    ros::Subscriber subObjectDetectCheck;
    int objectId;

    std::queue<DataFrame> pushedFrames;
    int id;
    DataFrame currentDataframe;
    bool next = false;

    float colorFOV = 71;
    float distanceIgnorePoint = 0.1;
    float outlineTarget = 0.1;
    float angleDeadzone = 0.0;
    CameraInfo cameraInfo;
    std::string laserFrame;
    std::string baseLinkFrame;

    geometry_msgs::Pose currentPose;
    std::vector<geometry_msgs::PointStamped> targetPositions;
    geometry_msgs::Pose lastKnownPos;
    float currentYaw;

    void CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes);
    void CallbackYoloObjects(const darknet_ros_msgs::ObjectCount::ConstPtr& objects);
    void CallBackOdom(const nav_msgs::Odometry::ConstPtr& odom);
    bool SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res);

    limo_yolo::map ConvertToLidar(const LaserScan& laser, const float& startAngleObject, const float& endAngleObject, bool checkPrevTarget = false, const std::vector<geometry_msgs::PointStamped>& target = std::vector<geometry_msgs::PointStamped>());
    void PublishAndMap(const DataFrame& dataframe, float minAngle= __FLT_MAX__, float maxAngle = __FLT_MAX__);
    std::vector<geometry_msgs::PointStamped> UpdateTargetPositions(const geometry_msgs::Point& transformation, float yawRotation);

    float QuaternionToYaw(const geometry_msgs::Quaternion& quaternion);
    float PixelToRad(float pixel);
    float DistanceSquare(const geometry_msgs::PointStamped & a, const geometry_msgs::PointStamped & b);
public:
    YoloProjection(const ros::NodeHandle& nodehandle);
    void CallbackImageAndLidar(const Image& image, const LaserScan& laser);
};