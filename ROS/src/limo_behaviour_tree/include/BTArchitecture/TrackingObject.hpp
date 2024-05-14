#include "Node.h"
#include "CreatePath.h"
#include "ObjectFound.h"
#include "TrackObject.h"

using namespace BehaviourTree;

class TrackingObject: public Node
{
private:
    TrackObject trackObject;
    ObjectFound objectFound;
    CreatePath createPath;
public:
    TrackingObject(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};

TrackingObject::TrackingObject(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub)
 :Node(nodehandle, logPub), trackObject(nodehandle, logPub), objectFound(nodehandle, logPub),createPath(nodehandle, logPub)
{
}
NodeStatus TrackingObject::Tick()
{
    NodeStatus x = trackObject.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    x = objectFound.Tick();
    if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x; 
    return createPath.Tick();   
}
