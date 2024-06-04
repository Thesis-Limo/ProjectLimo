#include "position.hpp"
#include <limo_motion_controller/OverrideMotion.h>
#include <Node.h>
using namespace BehaviourTree;

class CheckPath: public Node
{
private:
    ros::ServiceClient pathService; 
  ros::ServiceClient controllerClient;

public:
    CheckPath(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};