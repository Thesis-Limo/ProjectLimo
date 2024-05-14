#include <ros/ros.h>
#include <limo_motion_controller/OverrideMotion.h>
#include <Node.h>

using namespace BehaviourTree;
class MoveBack: public Node
{
private:
  ros::ServiceClient MoveBackService;

  limo_motion_controller::OverrideMotion moveMsg;

  float durationGoingBack;
  float speedGoingBack;
public:
  MoveBack(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
