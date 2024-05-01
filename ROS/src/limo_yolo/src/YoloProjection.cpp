#include "YoloProjection.h"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

YoloProjection::YoloProjection(const ros::NodeHandle& nodehandle):
    listenerTransform(buffer), nh(nodehandle), id(0)
{
    yoloImagePub = nh.advertise<Image>("/camera/yolo_input",10);
    posObjectPub = nh.advertise<geometry_msgs::PointStamped>("/ObjectPos",10);
    targetService = nh.advertiseService("/TrackID", &YoloProjection::SwitchTarget, this);
    sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&YoloProjection::CallbackYoloResult, this);

    //Get first camerainformation, for transformation
    cameraInfo = *(ros::topic::waitForMessage<CameraInfo>("/camera/rgb/camera_info"));
    K = cv::Mat(3, 3, CV_64F, (void*)cameraInfo.K.data());

    //Param
    baseLinkFrame = nh.param<std::string>("baseLinkFrame",std::string("base_link"));
    cameraFrame = nh.param<std::string>("cameraFrame",std::string("camera_rgb_optical_frame"));
    laserFrame = nh.param<std::string>("laserFrame",std::string("laser_link"));
    std::cout << laserFrame<<"\n";
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
{
    // To get the value of duration use the count()
    DataFrame img = this->pushedFrames.front();
    if(this->pushedFrames.size() <= 0) return;
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
        std::cout << "test\n";
        std::vector<Point3D> pixels = ConvertToLidar(img.lidar);

        std::cout << pixels[0].x<< "-\n";
        posObjectPub.publish(pixels[150].ConvertToPoseStamped());
        try {
            cv::Mat cv_image = cv_bridge::toCvCopy(img.currentImage, sensor_msgs::image_encodings::BGR8)->image;
            for (int i = 0; i < pixels.size(); i++)
            {
                if (0 <= pixels[i].x && 0 <= pixels[i].x < cv_image.cols && 0 <= pixels[i].x && 0 <= pixels[i].x < cv_image.rows)
                {
                    std::cout << pixels[i].x<< "-";
                    cv::circle(cv_image, cv::Point(pixels[i].x, pixels[i].y), 5,cv::Scalar(0, 0, 255) , -1);
                }
            }

            std::cout << "\n";
            cv::imshow("LIDAR Camera Overlay", cv_image);
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
        
    // member function on the duration object
    darknet_ros_msgs::BoundingBox *bounding_boxes=new darknet_ros_msgs::BoundingBox[end(msg->bounding_boxes)-begin(msg->bounding_boxes)];
    memcpy(bounding_boxes,&(msg->bounding_boxes[0]),(end(msg->bounding_boxes)-begin(msg->bounding_boxes))*sizeof(darknet_ros_msgs::BoundingBox));
    
    for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        if(bounding_boxes[i].id != objectId) continue;
        std::cout << "object found\n";
    }
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
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        float distance = laser.ranges[i];
        if(min_dist < distance && max_dist > distance)
        {
            float angle = laser.angle_min + i * laser.angle_increment;
            Point3D p{distance * cos(angle), distance * sin(angle),0};
            geometry_msgs::PointStamped lidar_point;
            lidar_point.header.frame_id = "laser_link";
            lidar_point.point.x = p.x - 0.020;
            lidar_point.point.y = p.y - 0.045;
            lidar_point.point.z = 0;
            //geometry_msgs::TransformStamped laserToBaseLink;
            try
            {
                geometry_msgs::PointStamped transformed_point;
                buffer.transform(lidar_point, transformed_point, "camera_link", ros::Duration(1.0));

                cv::Point3d pt(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
                cv::Mat pixel = K * cv::Mat(pt);
                //pixel /= pixel.at<float>(2, 0);

                // laserToBaseLink = buffer.lookupTransform(laserFrame,baseLinkFrame, ros::Time(0), ros::Duration(1.0));
                // geometry_msgs::PointStamped transformedPoint;
                // tf2::doTransform(p.ConvertToPoseStamped(),transformedPoint,laserToBaseLink);
                // cv::Mat pt = (cv::Mat_<double>(3, 1) <<
                //           transformedPoint.point.x,
                //           transformedPoint.point.y,
                //           transformedPoint.point.z);

                // cv::Mat pixel = K * pt;
                //pixel /= pixel.at<double>(2, 0);
                pixels.push_back(Point3D(pixel.at<float>(0, 0),pixel.at<float>(1, 0),0));
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }    
        }
        
    }  
    return pixels;
}
// void convertPixelToBaseLink(int pixel_x, int pixel_y) {
//     try {
//         // Create a point in the camera frame using pixel coordinates
//         geometry_msgs::PointStamped camera_point;
//         camera_point.header.frame_id = "camera_rgb_optical_frame";
//         camera_point.point.x = pixel_x;
//         camera_point.point.y = pixel_y;
//         camera_point.point.z = 1.0; // Arbitrary depth

//         // Transform the point to base_link frame
//         geometry_msgs::PointStamped base_link_point;
//         tf_buffer.transform(camera_point, base_link_point, "base_link", ros::Duration(1.0));

//         // Print the transformed point in the base_link frame
//         ROS_INFO("Base_link coordinates: (%f, %f, %f)",
//                     base_link_point.point.x, base_link_point.point.y, base_link_point.point.z);

//     } catch (tf2::TransformException& ex) {
//         ROS_WARN("Could not transform pixel to base_link: %s", ex.what());
//     }
// }