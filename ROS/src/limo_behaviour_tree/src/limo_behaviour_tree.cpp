#include <ros/ros.h>
#include <ros/package.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "Brake.h"
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


int main(int argc, char* argv[])
{
    ros::init(argc, argv,"limo_behaviour_tree");

    ros::NodeHandle nh("");
    BehaviorTreeFactory factory;

    factory.registerNodeType<Brake>("Brake");
    factory.registerNodeType<EmergencyBrake>("EmergencyBrake");
    factory.registerNodeType<MoveBack>("MoveBack");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<RotateAround>("RotateAround");
    factory.registerNodeType<TrackObject>("TrackObject");
    factory.registerNodeType<BatteryCheck>("BatteryCheck");
    factory.registerNodeType<CloseEnoughToTarget>("CloseEnoughToTarget");
    factory.registerNodeType<MinDistance>("MinDistance");
    factory.registerNodeType<SpeedNotZero>("SpeedNotZero");
    factory.registerNodeType<SpeedZero>("SpeedZero");
    factory.registerNodeType<TargetNotFound>("TargetNotFound");
    factory.registerNodeType<ToCloseToTarget>("ToCloseToTarget");

    factory.registerBehaviorTreeFromFile(ros::package::getPath("limo_behaviour_tree") + "/tree.xml");
    auto tree = factory.createTree("MainTree");
    auto visitor = [nh](TreeNode* node)
    {
        if (auto currentNode = dynamic_cast<Brake*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<EmergencyBrake*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<TrackObject*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<BatteryCheck*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<CloseEnoughToTarget*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<MinDistance*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<SpeedNotZero*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<SpeedZero*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<ToCloseToTarget*>(node))
            currentNode->Initialize(nh);
        else if (auto currentNode = dynamic_cast<MoveToTarget*>(node))
            currentNode->Initialize(nh);
    };
    BT::applyRecursiveVisitor(tree.rootNode(),visitor);
    // Apply the visitor to ALL the nodes of the tree
    ros::Rate r(10); // 10 hz

    NodeStatus status = NodeStatus::RUNNING;
    while (ros::ok())
    {
        ros::spinOnce();
        // Run the Behavior Tree
        tree.tickRoot();
        r.sleep();
    }
    return 0;
}