#include "Controller.h"
#include <cmath>

Controller::Controller(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    subMovement = nh.subscribe<limo_motion_controller::movementController>("/limo_movement",100,&Controller::CallBackMovement, this);
    pubCmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    minSpeed = nh.param<float>("minSpeed",-1.2);
    maxSpeed = nh.param<float>("maxSpeed",1.2);
    wheelBase = nh.param<float>("wheelBase",0.2);
    currentMovement.linear.x = 0;
    currentMovement.linear.y = 0;
    currentMovement.linear.z = 0;
    currentMovement.angular.x = 0;
    currentMovement.angular.y = 0;
    currentMovement.angular.z = 0;
}

void Controller::CallBackMovement(const limo_motion_controller::movementController::ConstPtr& msg)
{
    //calculating the turning radius and sending information to the cmd_vel 
    float delta = msg->angle;
    float v_l = remapSpeed(msg->speed);
    float v_a = (v_l * tan(delta))/ wheelBase;
    currentMovement.linear.x = v_l;
    currentMovement.angular.z = v_a;
}
void Controller::UpdateMovement()
{
    pubCmd.publish(currentMovement);
}
float Controller::remapSpeed(float speed)
{
    float max = 1;
    float min = 0;
    if (speed > 0)
        return  0 + (speed - 0) * (maxSpeed-0)/(max-min);
    return -1 * (speed - 0) * (minSpeed-0)/(max- min);
}
