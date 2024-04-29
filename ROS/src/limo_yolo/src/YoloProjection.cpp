#include "YoloProjection.h"

YoloProjection::YoloProjection(const ros::NodeHandle& nodehandle):
    listenerTransform(buffer), nh(nodehandle)
{
    yoloImagePub = nh.advertise<Image>("/camera/yolo_input",10);
    posObjectPub = nh.advertise<geometry_msgs::Point>("/ObjectPos",10);
    targetService = nh.advertiseService("/TrackID", &YoloProjection::SwitchTarget, this);
    sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&YoloProjection::CallbackYoloResult, this);

    //Get first camerainformation, for transformation
    cameraInfo = *(ros::topic::waitForMessage<CameraInfo>("/camera/rgb/camera_info"));
    K = cv::Mat(3, 3, CV_64F, (void*)cameraInfo.K.data());
    gotInfoBackFromYolo=true;
    
}

void YoloProjection::CallbackImageAndLidar(const Image& image, const LaserScan& laser)
/*
 * Callback for simultanious lidar and image information

    Args:
        image_msg (sensor_msgs/Image): current image
        lidar_msg (sensor_msgs/LaserScan): currentlaserscan
*/
{
    
    if(gotInfoBackFromYolo)
    {
        currentImage = image.data;
        std::cout << sizeof(currentImage) << " dim\n";
        currentLidarScan = laser;
        gotInfoBackFromYolo = false;
        yoloImagePub.publish(image);
    }   
}
void YoloProjection::CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    std::cout << " getinfo\n";

    darknet_ros_msgs::BoundingBox *bounding_boxes=new darknet_ros_msgs::BoundingBox[end(msg->bounding_boxes)-begin(msg->bounding_boxes)];
    memcpy(bounding_boxes,&(msg->bounding_boxes[0]),(end(msg->bounding_boxes)-begin(msg->bounding_boxes))*sizeof(darknet_ros_msgs::BoundingBox));
    
    for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        if(bounding_boxes[i].id != objectId) continue;
        std::cout << "object found\n";
    }
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    posObjectPub.publish(p);
    gotInfoBackFromYolo = true;
}

bool YoloProjection::SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res)
{
    objectId = (int)req.objectID;
    currentImage.clear();
}
