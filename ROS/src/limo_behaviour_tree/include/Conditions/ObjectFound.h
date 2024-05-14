#include "position.hpp"
#include "Node.h"
using namespace BehaviourTree;

class ObjectFound: public Node
{
private:
    ros::ServiceClient pathService;
public:
    ObjectFound(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};