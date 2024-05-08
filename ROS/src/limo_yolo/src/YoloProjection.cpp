#include "YoloProjection.h"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <darknet_ros_msgs/BoundingBox.h>
#include "limo_yolo/map.h"

YoloProjection::YoloProjection(const ros::NodeHandle& nodehandle):
    listenerTransform(buffer), nh(nodehandle), id(0)
{

    yoloImagePub = nh.advertise<Image>("/camera/yolo_input",10);
    mapPub = nh.advertise<limo_yolo::map>("/map",10);
    targetService = nh.advertiseService("/TrackID", &YoloProjection::SwitchTarget, this);
    sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&YoloProjection::CallbackYoloResult, this);
    std::string infoCamera = nh.param<std::string>("infoCamera", "/camera/rgb/camera_info");
    objectId = nh.param<float>("ObjectId", 39);
    baseLinkFrame = nh.param<std::string>("baseLinkFrame", "base_link");
    laserFrame = nh.param<std::string>("laserFrame", "laser_link");

    cameraInfo = *(ros::topic::waitForMessage<CameraInfo>(infoCamera));
    ROS_INFO("finishedInit with %i", objectId);
}

void YoloProjection::CallbackImageAndLidar(const Image& image, const LaserScan& laser)
/*
 * Callback for simultanious lidar and image information

    Args:
        image_msg (sensor_msgs/Image): current image
        lidar_msg (sensor_msgs/LaserScan): currentlaserscan
*/
{
    DataFrame newImg{id, laser, image};
    Image pubImage = image;
    pushedFrames.push(newImg);
    yoloImagePub.publish(pubImage);

    id++;
}
void YoloProjection::CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
/**
 * @brief convert the points from the yolo to a point in 3D base_link space
 * 
 */
{
    if(this->pushedFrames.size() <= 0) {
        return;
    }
    DataFrame img = this->pushedFrames.front();
    while(img.seq < msg->image_header.seq)
    {
        this->pushedFrames.pop();
        img = this->pushedFrames.front();
    }
    while(img.seq > msg->image_header.seq)
    {
        this->pushedFrames.pop();
        img = this->pushedFrames.front();
    }
    if (img.seq == msg->image_header.seq)
    {
        //Got the data from the yolo
        //std::vector<geometry_msgs::Point> pixels = ;
        std::vector<darknet_ros_msgs::BoundingBox> box;
        darknet_ros_msgs::BoundingBox *bounding_boxes=new darknet_ros_msgs::BoundingBox[end(msg->bounding_boxes)-begin(msg->bounding_boxes)];
        memcpy(bounding_boxes,&(msg->bounding_boxes[0]),(end(msg->bounding_boxes)-begin(msg->bounding_boxes))*sizeof(darknet_ros_msgs::BoundingBox));
    
        for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
            if(bounding_boxes[i].id != objectId) continue;
            if(box.size() > 0 && box[0].probability > bounding_boxes[i].probability) {
                box.clear();
                box.push_back(bounding_boxes[i]);
                }
            else {
                box.push_back(bounding_boxes[i]);
            }
        }
        if(box.size() < 0) return;
        float startRadObject = -M_PI;
        float endRadObject = -M_PI;
        float currentDistance = __FLT_MAX__;
        Point3D currentEndpoint;
        for (int i = 0; i < box.size(); i++)
        {
            //Calculate angle
            float pointX = box[i].xmin + (box[i].xmax - box[i].xmin)/2;
            float degPerPixel = colorFOV/cameraInfo.width;
            float totDeg = (pointX - cameraInfo.width/2)*degPerPixel;
            float rad = -totDeg /180 * M_PI;
            float id = (rad - img.lidar.angle_min) / img.lidar.angle_increment;
            id = round(id);

            if(currentDistance > img.lidar.ranges[id])
            {
                currentDistance = img.lidar.ranges[id];
                currentEndpoint = Point3D{currentDistance * cos(rad), currentDistance * sin(rad),0};
                float startpoint = box[i].xmin;
                float startTotDeg = (startpoint - cameraInfo.width/2)*degPerPixel;
                startRadObject = -startTotDeg /180 * M_PI;
                float endpoint = box[i].xmax;
                float endTotDeg = (endpoint - cameraInfo.width/2)*degPerPixel;
                endRadObject = -endTotDeg /180 * M_PI;
                ROS_INFO("JAJAJAJ %i", id);

            }            
        }   
        if(currentEndpoint.x == 0 || currentEndpoint.y == 0 || currentEndpoint.z == 0)
        {
            return;
        }
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = buffer.lookupTransform(
            baseLinkFrame, laserFrame,
            ros::Time(0));
        }  
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }   
            
        geometry_msgs::PointStamped p;
        p.header.frame_id = "laser_link";
        p.point.x = currentEndpoint.x;
        p.point.y = currentEndpoint.y;
        p.point.z = currentEndpoint.z;
        geometry_msgs::PointStamped transformed_point;

        tf2::doTransform(p, transformed_point, transformStamped);
        limo_yolo::map map;
        map.obstacles = this->ConvertToLidar(img.lidar, startRadObject, endRadObject);;
        map.goal = transformed_point;
        mapPub.publish(map);
        ROS_INFO("published");
    }
}

bool YoloProjection::SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res)
{
    objectId = (int)req.objectID;
}

const std::vector<geometry_msgs::PointStamped>& YoloProjection::ConvertToLidar(const LaserScan& laser, const float& startAngleObject, const float& endAngleObject)
{
    std::vector<geometry_msgs::PointStamped> pixels(laser.ranges.size());
    float min_dist = laser.angle_min;
    float max_dist = laser.angle_max;
    geometry_msgs::PointStamped transformed_point;
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        float distance = laser.ranges[i];
        if(min_dist < distance && max_dist > distance)
        {
            float angle = laser.angle_min + i * laser.angle_increment;
            if( angle >= startAngleObject && angle <= endAngleObject) continue;
            geometry_msgs::PointStamped lidar_point;
            lidar_point.header.frame_id = laserFrame;
            lidar_point.point.x = distance * cos(angle);
            lidar_point.point.y = distance * sin(angle);
            lidar_point.point.z = 0;
            try
            {
                buffer.transform(lidar_point, transformed_point, baseLinkFrame, ros::Duration(1.0));
                pixels.push_back(transformed_point);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }    
        }
    }  
    return pixels;
}