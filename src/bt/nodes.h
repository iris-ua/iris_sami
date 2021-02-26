#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace DummyNodes
{

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    srand (time(NULL));
    int fifty_fifty = rand() % 10;
    std::cout << "[ Battery: OK ]" << std::endl;
    return (fifty_fifty > 5) ?  BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    BT::NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

}
