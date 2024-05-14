#include "Node.h"
namespace BehaviourTree{
 Node::Node(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::string logInfo)
        :nh(nodehandle),logPub(logPub)
    {
    //     this->logInfo = []{
    // std_msgs::String ret;
    // ret.data = logInfo; 
    // return ret;}();
        this->logInfo.data = logInfo;
    }
    void Node::Log()
    {
        this->logPub.publish(logInfo);
    }
}