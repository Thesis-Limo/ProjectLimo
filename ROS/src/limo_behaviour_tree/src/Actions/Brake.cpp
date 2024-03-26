#include "Brake.h"
#include "std_srvs/Empty.h"
Brake::Brake(const std::string& name, const NodeConfiguration& conf)
    :ActionNodeBase(name, conf), nh("")
{
    client = nh.serviceClient<std_srvs::Empty>("/BT/Brake");
}
NodeStatus Brake::tick()
{
    std_srvs::Empty srv;
    std::cout << "Braking" << std::endl;
    if(client.call(srv))
        return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
}
void Brake::halt(){
  }