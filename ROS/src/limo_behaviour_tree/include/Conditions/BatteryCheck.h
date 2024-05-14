#include <limo_base/LimoStatus.h>
#include <Node.h>
using namespace BehaviourTree;

class BatteryCheck: public Node
{
private:
    int BatteryLevel = -1;
    float maxBatteryVoltage;
    float batteryToLow;
    ros::Subscriber sub;

    void BatteryCallBack(const limo_base::LimoStatus& msgs);
public:
    BatteryCheck(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub);
    NodeStatus Tick() override;
};