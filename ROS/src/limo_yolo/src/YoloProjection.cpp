#include "YoloProjection.h"
#include <cmath>    
#include <algorithm> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <darknet_ros_msgs/BoundingBox.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//TODO this is temp
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>


YoloProjection::YoloProjection(const ros::NodeHandle& nodehandle):
    listenerTransform(buffer), nh(nodehandle), id(0), angleDeadzone(0)
{

    yoloImagePub = nh.advertise<Image>("/camera/yolo_input",10);
    mapPub = nh.advertise<limo_yolo::map>("/map",10);
    //Debug purpose
    //mapPub = nh.advertise<geometry_msgs::PolygonStamped >("/map",10);
    //mapTempPub = nh.advertise<geometry_msgs::PolygonStamped >("/map2",10);
    targetService = nh.advertiseService("/TrackID", &YoloProjection::SwitchTarget, this);
    //Subscriber
    sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&YoloProjection::CallbackYoloResult, this);
    subObjectDetectCheck = nh.subscribe<darknet_ros_msgs::ObjectCount>("/darknet_ros/found_object",100,&YoloProjection::CallbackYoloObjects, this);
    subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &YoloProjection::CallBackOdom, this);
    serviceFoundObject = nh.advertiseService("/check_target", &YoloProjection::FoundObjectService, this);
    //param
    objectId = nh.param<float>("ObjectId", 39);
    baseLinkFrame = nh.param<std::string>("baseLinkFrame", "base_link");
    laserFrame = nh.param<std::string>("laserFrame", "laser_link");

    
    ROS_INFO("finishedInit yolo projection");
}

bool YoloProjection::FoundObjectService(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res)
{
    res.success = objectFound;
    return objectFound;
}

void YoloProjection::CallbackImageAndLidar(const Image& image, const LaserScan& laser)
/**
 * @brief Callback for simultanious lidar and image information
    @param image_msg (sensor_msgs/Image): current image
    @param lidar_msg (sensor_msgs/LaserScan): currentlaserscan
*/
{

    DataFrame newImg{id, laser, image, currentPose};
    Image pubImage = image;
    pushedFrames.push(newImg);
    yoloImagePub.publish(pubImage);
    id++;
}

void YoloProjection::CallbackYoloObjects(const darknet_ros_msgs::ObjectCount::ConstPtr& objects)
{
    
    if(this->pushedFrames.size() <= 0) {
        return;
    }
    DataFrame df = this->pushedFrames.front();
    while(df.seq < objects->image_header.seq)
    {
        this->pushedFrames.pop();
        df = this->pushedFrames.front();
    }
    while(df.seq > objects->image_header.seq)
    {
        this->pushedFrames.pop();
        df = this->pushedFrames.front();
    }
    if (df.seq == objects->image_header.seq)
    {
        this->currentDataframe = df;
    }
    if(objects->count <=0)
    { 
        objectFound = false;
        ROS_INFO("Nothing in sight");
        PublishAndMap(df);
        return;
    }
}


void YoloProjection::CallbackYoloResult(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
/**
 * @brief convert the points from the yolo to a point in 3D base_link space
 * @param msg holds information from the yolo, boudingboxes for each item
 */
{
    if (currentDataframe.seq == msg->image_header.seq)
    {
        //Got the data from the yolo
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
        if(box.size() <= 0) {
            objectFound = false;
            ROS_INFO("TargetNotFound");
            PublishAndMap(currentDataframe);
            return;
        }
        float startRadObject = -M_PI;
        float endRadObject = -M_PI;
        float currentDistance = __FLT_MAX__;
        for (int i = 0; i < box.size(); i++)
        {
            //Calculate angle
            float pointX = box[i].xmin + (box[i].xmax - box[i].xmin)/2;
            float rad = PixelToRad(pointX,currentDataframe.currentImage.width);
            float id = (rad - currentDataframe.lidar.angle_min) / currentDataframe.lidar.angle_increment;
            id = round(id);

            if(currentDistance > currentDataframe.lidar.ranges[id])
            {
                currentDistance = currentDataframe.lidar.ranges[id];
                startRadObject =  PixelToRad(box[i].xmax,currentDataframe.currentImage.width);
                endRadObject = PixelToRad(box[i].xmin, currentDataframe.currentImage.width);
                std::cout << currentDataframe.currentImage.width << "\n";
            }            
        }   
        if(currentDistance == __FLT_MAX__)
        {
            ROS_INFO("TargetNotInReach");
            objectFound = false;    
            PublishAndMap(currentDataframe);
            return;
        }    
        objectFound = true;    
        ROS_INFO("TargetFound");
        PublishAndMap(currentDataframe, startRadObject, endRadObject);
    }
} 

bool YoloProjection::SwitchTarget(limo_behaviour_tree::TypeObjectTracking::Request& req, limo_behaviour_tree::TypeObjectTracking::Response& res)
{
    objectId = (int)req.objectID;
}

limo_yolo::map YoloProjection::ConvertToLidar(const LaserScan& laser, const float& startAngleObject, const float& endAngleObject, bool checkPrevTarget, const std::vector<geometry_msgs::PointStamped>& target)
{
    std::vector<geometry_msgs::PointStamped> pixelsObstacles;
    std::vector<std::pair<float,geometry_msgs::PointStamped>> pixelsTargetWithDistance;
    limo_yolo::map map;
    float min_dist = laser.range_min;
    float max_dist = laser.range_max;
    int startId = 0;
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        float distance = laser.ranges[i];
        if(min_dist < distance && max_dist > distance)
        {
            float angle = laser.angle_min + i * laser.angle_increment;
            geometry_msgs::PointStamped lidar_point;
            geometry_msgs::PointStamped transformed_point;

            lidar_point.header.frame_id = laserFrame;
            lidar_point.point.x = distance * cos(angle);
            lidar_point.point.y = distance * sin(angle);
            lidar_point.point.z = 0;
            try
            {
                buffer.transform(lidar_point, transformed_point, baseLinkFrame, ros::Duration(1.0));
                if(angle >= startAngleObject && angle <= endAngleObject) 
                {
                    std::pair<float,geometry_msgs::PointStamped> p;
                    p.first = distance;
                    if(!checkPrevTarget)
                    {
                        p.second = transformed_point;
                        pixelsTargetWithDistance.push_back(p);
                    }
                    startId++;
                }
                else
                    pixelsObstacles.push_back(transformed_point);  
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }  
        }
    }  

    //Taking outliners out of the target if it exeeds a distance it becomes obstacle
    if(pixelsTargetWithDistance.size() <= 0)
    {
        map.obstacles = pixelsObstacles;
        return map; 
    }
    std::sort(pixelsTargetWithDistance.begin(), pixelsTargetWithDistance.end(), [](const auto& a,const auto& b){return a.first < b.first;});
    int id = round(pixelsTargetWithDistance.size()/2);
    float mean = pixelsTargetWithDistance[id].first;

    std::vector<geometry_msgs::PointStamped> out;
    std::vector<geometry_msgs::PointStamped> pixelsTarget;
    for (int i = 0; i < pixelsObstacles.size(); i++)
    {
        if(DistanceSquare(pixelsObstacles[i], pixelsTargetWithDistance[id].second) > 0.1)
        {
            out.push_back(pixelsObstacles[i]);            
        }
    }
    
    for (int i = 0; i < pixelsTargetWithDistance.size(); i++)
    {
        if(abs(mean - pixelsTargetWithDistance[i].first) < outlineTarget)
            pixelsTarget.push_back(pixelsTargetWithDistance[i].second);
        else 
            pixelsObstacles.push_back(pixelsTargetWithDistance[i].second);  
    }
    
    
    map.obstacles = out;
    map.goal = pixelsTarget;
    return map;
}

void YoloProjection::PublishAndMap(const DataFrame& dataframe, float minAngle, float maxAngle)
{
    limo_yolo::map map;
    if(minAngle == __FLT_MAX__ && targetPositions.size() == 0)
    {
        map = this->ConvertToLidar(dataframe.lidar, minAngle, maxAngle, false);
    }
    else if(minAngle == __FLT_MAX__)
    {
        //update previous position of target
        geometry_msgs::Point changePos;
        changePos.x = dataframe.currentPose.position.x - lastKnownPos.position.x;
        changePos.y = dataframe.currentPose.position.y - lastKnownPos.position.y;
        changePos.z = dataframe.currentPose.position.z - lastKnownPos.position.z;

        ROS_INFO("UpdatePosition");
        double yaw = QuaternionToYaw(dataframe.currentPose.orientation);
        yaw -= QuaternionToYaw(lastKnownPos.orientation); 
        auto updatedPos = UpdateTargetPositions(changePos, -yaw);
        
        //take previous target with updated position
        float min  = __FLT_MAX__;
        float max = -__FLT_MAX__;
        for (int i = 0; i < updatedPos.size(); i++)
        {
            float temp = atan2(updatedPos[i].point.y,updatedPos[i].point.x) + yaw;
            if(temp < min)
                min = temp;
            if(temp > max)
                max = temp;
        }
        std::cout << min << "-" << max <<"\n";
        //TODO testing
        map = this->ConvertToLidar(dataframe.lidar, min, max, true, updatedPos);
        map.goal = updatedPos;

    }
    else{
        std::cout << minAngle  << "-" << maxAngle   <<"\n";
        map = this->ConvertToLidar(dataframe.lidar, minAngle - angleDeadzone, maxAngle + angleDeadzone);
        targetPositions.clear();
        targetPositions = map.goal;
        lastKnownPos = dataframe.currentPose;
    }
    mapPub.publish(map);
    // geometry_msgs::PolygonStamped path;
    // path.header.frame_id = laserFrame;
    // std::vector<geometry_msgs::Point32> p;
    // for (int i = 0; i < map.goal.size(); i++)
    // {
    //     geometry_msgs::Point32 pe;
    //     pe.x = map.goal[i].point.x -0.105;
    //     pe.y = map.goal[i].point.y;
    //     pe.z = map.goal[i].point.z -0.08;
    //     p.push_back(pe);
    // }
    // path.polygon.points = p;
    // mapPub.publish(path);
    // //TODO temp
    // geometry_msgs::PolygonStamped path2;

    // path2.header.frame_id = laserFrame;
    // std::vector<geometry_msgs::Point32> pt;
    // for (int i = 0; i < map.obstacles.size(); i++)
    // {
    //     geometry_msgs::Point32 pe;
    //     pe.x = map.obstacles[i].point.x -0.105;
    //     pe.y = map.obstacles[i].point.y;
    //     pe.z = map.obstacles[i].point.z -0.08;
    //     pt.push_back(pe);
    // }
    // path2.polygon.points = pt;
    // mapTempPub.publish(path2);
    ROS_INFO("map published");
}

void YoloProjection::CallBackOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    currentPose = odom->pose.pose;   
}

float YoloProjection::PixelToRad(float pixel, float width)
/**
 * @brief convert a camerapixel to a deg based on cameraFOV
 */
{
    float deg = (pixel - width / 2) * (colorFOV / width);
    return -deg / 180 * M_PI;
}
std::vector<geometry_msgs::PointStamped> YoloProjection::UpdateTargetPositions(const geometry_msgs::Point& transformation, float yawRotation)
{
    std::vector<geometry_msgs::PointStamped> updatedPos;
    updatedPos.reserve(targetPositions.size());
    for (int i =0;i < targetPositions.size(); i++)
    {
        geometry_msgs::PointStamped newPos;
        newPos.point.x = targetPositions[i].point.x - (transformation.x) -0.105;
        newPos.point.y = targetPositions[i].point.y - (transformation.y);
        newPos.point.z = targetPositions[i].point.z - (transformation.z);
        float temp = newPos.point.x * cos(yawRotation) - newPos.point.y * sin(yawRotation) + 0.105;
        newPos.point.y = newPos.point.x * sin(yawRotation) + newPos.point.y * cos(yawRotation);
        newPos.point.x = temp;
        if(abs(newPos.point.x)  <= outlineTarget && abs(newPos.point.y) <= outlineTarget)
            continue;
        updatedPos.push_back(newPos);
    }
    return updatedPos;
}
float YoloProjection::QuaternionToYaw(const geometry_msgs::Quaternion& quaternion) {
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    double roll, pitch,yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    return yaw;
}

float YoloProjection::DistanceSquare(const geometry_msgs::PointStamped & a, const geometry_msgs::PointStamped & b)
{
    float x,y,z;
    x = abs(b.point.x - a.point.x); 
    y = abs(b.point.y - a.point.y); 
    z = abs(b.point.z - a.point.z); 
    return x*x + y*y + z*z;
}