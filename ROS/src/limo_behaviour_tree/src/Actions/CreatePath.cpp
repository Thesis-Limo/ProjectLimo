#include "CreatePath.h"
#include "limo_behaviour_tree/EndGoal.h"
CreatePath::CreatePath(const std::string& name, const NodeConfiguration& conf)
    :ActionNodeBase(name, conf)
{}
void CreatePath::Initialize(const ros::NodeHandle& nodehandle)
{
    nh = nodehandle;
    client = nh.serviceClient<limo_behaviour_tree::EndGoal>("/GoalPos");
}
NodeStatus CreatePath::tick()
/*
 * Gets information form the previous condition and calls function /GoalPos, so that it can calculate the trajectory
*/
{
    auto res = getInput<Point3D>("goal");
    if( !res )
    {
        throw RuntimeError("error reading port [target]:", res.error());
    }
    Point3D target = res.value();
    limo_behaviour_tree::EndGoal    srv;
    srv.request.goalPos.x = target.x;
    srv.request.goalPos.y = target.y;
    srv.request.goalPos.z = target.z;
    std::cout << target.x << std::endl;
    if(client.call(srv))
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}
void CreatePath::halt(){
  }