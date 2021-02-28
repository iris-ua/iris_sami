#include <ros/ros.h>
#include <ros/package.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// file that contains the custom nodes definitions
#include "ur10e_nodes.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "ur10e_tree");

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    using namespace UR10e;

    // Node registration
    factory.registerSimpleCondition("UR10eOnline", std::bind(isOnline));
    factory.registerSimpleAction("Pause", std::bind(Pause));
    factory.registerSimpleAction("Grip", std::bind(grip));
    factory.registerSimpleAction("Release", std::bind(release));

    factory.registerNodeType<Sleep>("Sleep");
    factory.registerNodeType<Velocity>("Velocity");
    factory.registerNodeType<Joints>("Joints");
    factory.registerNodeType<JointsAlias>("JointsAlias");
    factory.registerNodeType<Pose>("Pose");
    factory.registerNodeType<Move>("Move");

    std::string tree_path = ros::package::getPath("iris_sami") + "/trees/";
    auto tree = factory.createTreeFromFile(tree_path + argv[1]);
    
    printTreeRecursively(tree.rootNode());
    BT::StdCoutLogger logger_cout(tree);
    BT::PublisherZMQ publisher_zmq(tree);

    std::cout << "Behavior Tree Started" << std::endl;

    while( tree.tickRoot() == BT::NodeStatus::RUNNING)
    {
        sleep(0.01);
    }

    std::cout << "Behavior Tree Finished" << std::endl;
    
    sleep(1);

    return 0;
}