#include "Node.h"
using namespace BehaviourTree;

class Sequence: public Node
{
private:
    std::vector<Node*> children;
public:
    Sequence(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::vector<Node*>& children);
    NodeStatus Tick() override;
    ~Sequence();
};
