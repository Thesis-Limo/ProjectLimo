#include "YoloProjection.h"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/TransformStamped.h>

YoloProjection::YoloProjection(const ros::NodeHandle& nodehandle):
    listenerTransform(buffer), nh(nodehandle), id(0)
{
    yoloImagePub = nh.advertise<Image>("/camera/yolo_input",10);
    posObjectPub = nh.advertise<geometry_msgs::PointStamped>("/ObjectPos",10);
    targetService = nh.advertiseService("/TrackID", &YoloProjection::SwitchTarget, this);
    sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&YoloProjection::CallbackYoloResult, this);

    //Get first camerainformation, for transformation
    CameraInfo cameraInfo = *(ros::topic::waitForMessage<CameraInfo>("/camera/rgb/camera_info"));
    cam_model_.fromCameraInfo(cameraInfo);

    //Param
    baseLinkFrame = nh.param<std::string>("baseLinkFrame",std::string("base_link"));
    cameraFrame = nh.param<std::string>("cameraFrame",std::string("camera_rgb_optical_frame"));
    laserFrame = nh.param<std::string>("laserFrame",std::string("laser_link"));
    std::cout << laserFrame<<"\n";
}

void YoloProjection::CallbackImageAndLidar(const Image& image, const limo_yolo::transformedLidarPoints& laser)
/*
 * Callback for simultanious lidar and image information

    Args:
        image_msg (sensor_msgs/Image): current image
        lidar_msg (sensor_msgs/LaserScan): currentlaserscan
*/
{
    //DataFrame newImg{id, nullptr, image};
    Image pubImage = image;
    //pushedFrames.push(newImg);
    yoloImagePub.publish(pubImage);
    id++;
}
void YoloProjection::CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    // To get the value of duration use the count()

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
        //std::cout << img.lidar.points[0].x << "\n";
        return;
        // darknet_ros_msgs::BoundingBox *bounding_boxes=new darknet_ros_msgs::BoundingBox[end(msg->bounding_boxes)-begin(msg->bounding_boxes)];
        // memcpy(bounding_boxes,&(msg->bounding_boxes[0]),(end(msg->bounding_boxes)-begin(msg->bounding_boxes))*sizeof(darknet_ros_msgs::BoundingBox));
    
        // for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        //     if(bounding_boxes[i].id != objectId) continue;
        //     std::cout << "object found\n";
        // }
        // try {
        //     cv::Mat cv_image = cv_bridge::toCvCopy(img.currentImage, sensor_msgs::image_encodings::BGR8)->image;
        //     for (int i = 0; i < pixels.size(); i++)
        //     {
        //         if (0 <= pixels[i].x && pixels[i].x < cv_image.cols && 0 <= pixels[i].y && pixels[i].y < cv_image.rows)
        //         {
        //             cv::circle(cv_image, cv::Point(pixels[i].x, pixels[i].y), 3,cv::Scalar(0, 0, 255) , -1);
        //         }
        //     }
        //     cv::imshow("LIDAR Camera Overlay", cv_image);
        //     cv::waitKey(1);
        // } catch (cv_bridge::Exception& e) {
        //     ROS_ERROR("cv_bridge exception: %s", e.what());
        //     return;
        // }
    }
        
    // member function on the duration object
    
    // geometry_msgs::Point p;
    // p.x = 0;
    // p.y = 0;
    // p.z = 0;
    // posObjectPub.publish(p);
    // std::cout << " published\n";

    // gotInfoBackFromYolo = true;
}

bool YoloProjection::SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res)
{
    objectId = (int)req.objectID;
    //currentImage.clear();
}


std::vector<Point3D> YoloProjection::ConvertToLidar(const LaserScan& laser)
{
    std::vector<Point3D> pixels;
    float min_dist = laser.angle_min;
    float max_dist = laser.angle_max;
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = buffer.lookupTransform(
            cameraFrame, laserFrame,
            ros::Time(0));
    }  catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }    
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        float distance = laser.ranges[i];
        if(min_dist < distance && max_dist > distance)
        {
            float angle = laser.angle_min + i * laser.angle_increment;
            if(abs(angle) > 0.654) continue;
            Point3D p{distance * cos(angle), distance * sin(angle),0};
            geometry_msgs::PointStamped lidar_point;
            lidar_point.header.frame_id = laserFrame;
            lidar_point.point.x = p.x;
            lidar_point.point.y = p.y;
            lidar_point.point.z = 0;
            try
            {
                geometry_msgs::PointStamped transformed_point;
                tf2::doTransform(lidar_point, transformed_point, transformStamped);
                //buffer.transform(lidar_point, transformed_point, cameraFrame, ros::Duration(1.0));
                cv::Point3d pt(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
                cv::Point2d newPoint = cam_model_.project3dToPixel(pt);
                pixels.push_back(Point3D(newPoint.x, newPoint.y,0));
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }    
        }
    }  
    return pixels;
}