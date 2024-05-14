#include "Node.h"
using namespace BehaviourTree;

class Fallback: public Node
{
private:
    std::vector<Node*> children;
public:
    Fallback(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::vector<Node*>& children);
    NodeStatus Tick() override;
    ~Fallback();
};
