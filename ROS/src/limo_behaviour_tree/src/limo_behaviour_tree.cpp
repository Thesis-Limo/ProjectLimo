#include <ros/ros.h>
#include <ros/package.h>
// #include "behaviortree_cpp_v3/bt_factory.h"
#include "EmergencyBrake.h"
#include "MoveBack.h"
#include "MoveToTarget.h"
#include "RotateAround.h"
#include "TrackObject.h"
#include "BatteryCheck.h"
#include "CloseEnoughToTarget.h"
#include "MinDistance.h"
#include "SpeedNotZero.h"
#include "SpeedZero.h"
#include "TargetNotFound.h"
#include "ToCloseToTarget.h"
#include "ObjectFound.h"
#include "CheckPath.h"
#include "CreatePath.h"
#include <std_msgs/String.h>
#include "Fallback.h"
#include "Sequence.h"
#include "Brake.h"
using namespace BehaviourTree;

Node* InitTree(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub)
{
    Brake* brake = new Brake(nodehandle, logPub);
    EmergencyBrake* emergencyBrake = new EmergencyBrake(nodehandle,duration, logPub);
    MoveBack* moveBack = new MoveBack(nodehandle, logPub);
    MoveToTarget* moveToTarget = new MoveToTarget(nodehandle, logPub);
    RotateAround* rotateAround = new RotateAround(nodehandle, logPub);
    TrackObject* trackObject = new TrackObject(nodehandle, logPub);
    BatteryCheck* batteryCheck = new BatteryCheck(nodehandle, logPub);
    MinDistance* minDistance = new MinDistance(nodehandle, logPub);
    TargetNotFound* targetNotFound = new TargetNotFound(nodehandle, logPub);
    CheckPath* checkPath = new CheckPath(nodehandle, logPub);
    //Setup Tree
    std::vector<Node*> em;
    em.push_back(moveBack);
    Sequence* DistanceBased = new Sequence(nodehandle, logPub,em);
    em.clear();
    em.push_back(minDistance);
    em.push_back(DistanceBased);
    Sequence* emergency = new Sequence(nodehandle, logPub,em);
    em.clear();
    em.push_back(targetNotFound);
    em.push_back(rotateAround);
    Sequence* targetsearch = new Sequence(nodehandle, logPub,em);
    em.clear();
    em.push_back(checkPath);
    em.push_back(targetsearch);
    Sequence* targetCheck = new Sequence(nodehandle, logPub,em);
    em.clear();
    em.push_back(emergency);
    em.push_back(targetCheck);
    em.push_back(moveToTarget);
    Fallback* movementRobot = new Fallback(nodehandle, logPub,em);
    em.clear();
    em.push_back(batteryCheck);
    em.push_back(brake);
    Fallback* checkBattery = new Fallback(nodehandle, logPub,em);
    em.clear();
    em.push_back(checkBattery);
    em.push_back(movementRobot);
    Sequence* root = new Sequence(nodehandle, logPub,em);
    return root;
}
Node* InitTreeSearch(const ros::NodeHandle& nodehandle, float duration, const ros::Publisher& logPub)
{
    TrackObject* trackObject = new TrackObject(nodehandle, logPub);
    ObjectFound* objectFound = new ObjectFound(nodehandle, logPub);
    CreatePath* createPath = new CreatePath(nodehandle, logPub);
    //Setup Tree
    std::vector<Node*> em;
    em.push_back(trackObject);
    em.push_back(objectFound);
    em.push_back(createPath);
    Sequence* TrackObject = new Sequence(nodehandle, logPub,em);
    return TrackObject;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_behaviour_tree");

    ros::NodeHandle nh("~");
    //Logging
    ros::Publisher logger = nh.advertise<std_msgs::String>("/BT/Log", 100);
    //BT
    Node* root = InitTree(nh, 1,logger);
    Node* rootSearch = InitTreeSearch(nh, 1,logger);
    ros::Rate r(10); // 10 hz
    float sec = 1;
  
    NodeStatus status = NodeStatus::RUNNING;
    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        i++;
        root->Tick();
        rootSearch->Tick();
        // Run the Behavior Tree
        //tree.tickRoot();
        //treeTracking.tickRoot();
        r.sleep();
    }
    return 0;
}