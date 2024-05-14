#include "position.hpp"
#include <Node.h>
using namespace BehaviourTree;

class CheckPath: public Node
{
private:
    ros::ServiceClient pathService;    
public:
    CheckPath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};