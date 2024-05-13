#include <ros/ros.h>
#include <ros/package.h>
#include "Controller.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_motion_controller");

    ros::NodeHandle nh("~");
    ros::Rate r(10); // 10 hz
    Controller c = Controller(nh);
    while (ros::ok())
    {
        c.UpdateMovement();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}