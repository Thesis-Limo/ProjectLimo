#include "Controller.h"
#include <cmath>

Controller::Controller(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    subMovement = nh.subscribe<limo_motion_controller::MovementController>("/limo_movement",100,&Controller::CallBackMovement, this);
    subMotionPlan = nh.subscribe<limo_motion_controller::MotionPlan>("/limo_motionplan",100, &Controller::CallBackMotionPlan, this);
    pubCmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    minSpeed = nh.param<float>("minSpeed",-1.2);
    maxSpeed = nh.param<float>("maxSpeed",1.2);
    wheelBase = nh.param<float>("wheelBase",0.2);
    backUpMotion = new Motion{0,0,-1,0,0,0};
}

void Controller::CallBackMovement(const limo_motion_controller::MovementController::ConstPtr& msg)
{
    //Overwriting the currentMotionPlan
    //calculating the turning radius and sending information to the cmd_vel 
    overrideMotionPlan = new Motion{msg->angle, msg->speed, msg->duration,currentSpeed, currentSteeringAngle,0};
    if(motionPlan.size() > 0)
    {
        Motion* currentM = motionPlan.front();
        currentM->currentDuration = currentTime;
        currentTime = 0;
    }
}
void Controller::CallBackMotionPlan(const limo_motion_controller::MotionPlan::ConstPtr& msg)
{
    ROS_INFO("AddMotionPlan");
    while (motionPlan.size() > 0)
    {
        delete motionPlan.front();
        motionPlan.pop();
    }

    currentTime = 0;
    for (int i = 0; i < msg->sequence.size(); i++)
    {
        Motion* m = new Motion{msg->sequence[i].angle,msg->sequence[i].speed, msg->sequence[i].duration, 0,0,0};
        motionPlan.push(m);
    }
    UpdateMovement();
}

void Controller::UpdateMovement()
{
    Motion* currentM;
    ros::Time currentTimeStamp = ros::Time::now();
    ros::Duration d = (currentTimeStamp - prevTimestamp);

    if(overrideMotionPlan == nullptr)
    {
        if (motionPlan.size() <=0)
            currentM = backUpMotion;
        else {
            currentM = motionPlan.front();
        }
        if(currentM->duration < currentTime + d.toSec() && currentM->duration != -1)
        {
            currentTime = currentTime - currentM->duration;
            motionPlan.pop();
            ROS_LOG("Next");
            delete currentM;
            if (motionPlan.size() <=0)
                currentM = backUpMotion;
            else {
                currentM = motionPlan.front();
                currentM->startAngle = currentSteeringAngle;
                currentM->startSpeed = currentSpeed;
            }
        }
    }
    else 
    {
        if (overrideMotionPlan->duration < currentTime + d.toSec() && overrideMotionPlan->duration != -1)
        {
            currentTime = currentTime - overrideMotionPlan->duration;
             if (motionPlan.size() <=0)
                currentM = backUpMotion;
            else {
                currentM = motionPlan.front();
                currentM->startAngle = currentSteeringAngle;
                currentM->startSpeed = currentSpeed;
            }
            ROS_INFO("delete override");
            delete overrideMotionPlan;
            overrideMotionPlan = nullptr;
        }
        else
        {
            currentM = overrideMotionPlan;
        } 
    }
    currentTime += d.toSec();
    pubCmd.publish(CalculateMovement(currentM));
    prevTimestamp = currentTimeStamp;
}

float Controller::remapSpeed(float speed)
{
    float max = 1;
    float min = 0;
    if (speed > 0)
        return  0 + (speed - 0) * (maxSpeed-0)/(max-min);
    return -1 * (speed - 0) * (minSpeed-0)/(max- min);
}

geometry_msgs::Twist Controller::CalculateMovement(const Motion* currentMotion)
{
    geometry_msgs::Twist currentMovement;
    float currentSpeed = (currentMotion->speed - currentMotion->startSpeed)/(currentMotion->duration) * currentTime + currentMotion->startSpeed;
    float currentAngle = (currentMotion->angle - currentMotion->startAngle)/(currentMotion->duration) * currentTime + currentMotion->startAngle;
    

    this->currentSteeringAngle = currentAngle;
    this->currentSpeed = currentSpeed;
    float v_l = remapSpeed(currentSpeed);
    float v_a = (v_l * tan(currentAngle))/ wheelBase;
    currentMovement.linear.x = v_l;
    currentMovement.angular.z = v_a;
    return currentMovement;
}
