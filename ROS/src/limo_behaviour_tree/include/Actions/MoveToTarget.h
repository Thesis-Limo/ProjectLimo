#include <limo_motion_controller/OverrideMotion.h>
#include <geometry_msgs/Twist.h>
#include <Node.h>
using namespace BehaviourTree;
class MoveToTarget: public Node
{
private:
  ros::Publisher movePub;
  ros::ServiceClient client;
public:
  MoveToTarget(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
