#include "Brake.h"
#include "std_srvs/Empty.h"
Brake::Brake(const std::string& name, const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(name, conf), nh("")
{
    client = nh.serviceClient<std_srvs::Empty>("/BT/Brake");
}
BT::NodeStatus Brake::tick()
{
    std_srvs::Empty srv;
    std::cout << "Braking" << std::endl;
    if(client.call(srv))
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::FAILURE;
}
void Brake::halt(){
  }