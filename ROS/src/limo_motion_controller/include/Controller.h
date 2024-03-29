#include <ros/ros.h>
#include "limo_motion_controller/movementController.h"
#include <geometry_msgs/Twist.h>

class Controller
{
private:
    float remapSpeed(float speed);
    void CallBackMovement(const limo_motion_controller::movementController::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Subscriber subMovement;
    ros::Publisher pubCmd;

    float wheelBase = 0.2; //meters
    float minSpeed = -1.2;
    float maxSpeed = 1.2;
    geometry_msgs::Twist currentMovement;

public:
    Controller(const ros::NodeHandle& nodehandle);
    void UpdateMovement();
};