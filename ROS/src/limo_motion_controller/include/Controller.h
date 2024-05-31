#include <ros/ros.h>
#include "limo_motion_controller/MovementController.h"
#include "limo_motion_controller/MotionPlan.h"
#include "limo_motion_controller/OverrideMotion.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <queue>
#include <string>

struct Motion{
    float angle;
    float speed;
    float duration;
    float startSpeed;
    float startAngle;
    float currentDuration;
};
class Controller
{
private:
    float remapSpeed(float speed);
    void CallBackMovement(const limo_motion_controller::MovementController::ConstPtr& msg);
    bool ServiceCallBackMovement(limo_motion_controller::OverrideMotion::Request& req,limo_motion_controller::OverrideMotion::Response& response);
    void CallBackMotionPlan(const limo_motion_controller::MotionPlan::ConstPtr& msg);
    void CallBackMotionPlanAppend(const limo_motion_controller::MovementController::ConstPtr &msg); 

    ros::NodeHandle nh;
    ros::Subscriber subMovement;
    ros::Subscriber subMotionPlan;
    ros::Subscriber subMotionPlanAppend;
    ros::ServiceServer serviceOverride;
    ros::Publisher pubCmd;

    ros::Time prevTimestamp;
    float currentSpeed = 0;

    float wheelBase = 0.2; //meters
    float minSpeed = -1.2;
    float maxSpeed = 1.2;
    float currentTime = 0;
    float currentSteeringAngle = 0; //in rad

    std::queue<Motion*> motionPlan; 
    Motion* overrideMotionPlan;
    Motion* backUpMotion;
    geometry_msgs::Twist CalculateMovement(const Motion* currentMotion);

public:
    Controller(const ros::NodeHandle& nodehandle);
    void UpdateMovement();
};