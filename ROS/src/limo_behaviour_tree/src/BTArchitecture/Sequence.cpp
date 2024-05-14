#include "Sequence.h"

Sequence::Sequence(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::vector<Node*>& children)
  :Node(nodehandle, logPub)
{
    this->children = children;
}

Sequence::~Sequence()
{
    for (int i = children.size()-1; i >=0 ; i--)
    {
        delete children[i];
    }
    children.clear();
}

NodeStatus Sequence::Tick()
{
    for (int i = 0; i < children.size(); i++)
    {
        NodeStatus x = children[i]->Tick();
        if((x == NodeStatus::FAILURE) || (x == NodeStatus::RUNNING)) return x;       
    }
    return NodeStatus::SUCCESS;
}
