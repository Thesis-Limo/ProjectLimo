#include <limo_motion_controller/MovementController.h>
#include <limo_motion_controller/OverrideMotion.h>
#include <Node.h>

using namespace BehaviourTree;
class Brake: public Node
{
private:
  ros::ServiceClient brakeService;
  limo_motion_controller::OverrideMotion brakeMsg;
  float durationSlowDown = 1;
public:
  Brake(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
  NodeStatus Tick() override;
};
