#include "Fallback.h"

Fallback::Fallback(const ros::NodeHandle& nodehandle, const ros::Publisher& logPub, std::vector<Node*>& children)
  :Node(nodehandle, logPub)
{
    this->children = children;
}

Fallback::~Fallback()
{
    for (int i = children.size()-1; i >=0 ; i--)
    {
        delete children[i];
    }
    children.clear();
}

NodeStatus Fallback::Tick()
{
    for (int i = 0; i < children.size(); i++)
    {
        NodeStatus x = children[i]->Tick();
        if(x == NodeStatus::SUCCESS || x == NodeStatus::RUNNING) return x;       
    }
    return NodeStatus::FAILURE;
}
