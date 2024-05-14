
#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#pragma once

namespace BehaviourTree{

    enum class NodeStatus{
        SUCCESS,FAILURE,RUNNING
    };
    class Node
    {
    private:
        ros::Publisher logPub;
        std_msgs::String logInfo;
    protected:
        void Log();
        ros::NodeHandle nh;
    public:
        Node(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::string logInfo="");
        virtual NodeStatus Tick() = 0;

    };
}
