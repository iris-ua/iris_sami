#include <ros/ros.h>
#include <ros/package.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

// file that contains the custom nodes definitions
#include "ur10e_nodes.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "ur10e_tree");

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    using namespace UR10e;

    // Node regostration
    factory.registerSimpleCondition("UR10eOnline", std::bind(isOnline));
    factory.registerSimpleAction("Grip", std::bind(grip));
    factory.registerSimpleAction("Release", std::bind(release));

    factory.registerNodeType<Velocity>("Velocity");
    factory.registerNodeType<Joints>("Joints");
    factory.registerNodeType<Pose>("Pose");
    factory.registerNodeType<Move>("Move");

    std::string tree_path = ros::package::getPath("iris_sami") + "/trees/";
    auto tree = factory.createTreeFromFile(tree_path + "ur10e.xml");

    BT::PublisherZMQ publisher_zmq(tree);

    while (true)
    {
        tree.tickRoot();
        break;
    }

    std::cout << "Behavior Tree Finished" << std::endl;

    sleep(2);

    return 0;
}