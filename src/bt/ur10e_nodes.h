#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/String.h>
#include "iris_sami/Status.h"
#include "iris_sami/Velocity.h"
#include "iris_sami/NoArguments.h"
#include "iris_sami/JointGoal.h"
#include "iris_sami/JointGoalName.h"
#include "iris_sami/PoseGoal.h"
#include "iris_sami/RelativeMove.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace UR10e
{
    BT::NodeStatus isOnline()
    {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<iris_sami::Status>("iris_sami/status");
        iris_sami::Status srv;
        if (client.call(srv))
        {
            std::cout << srv.response.feedback << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "Failed to call service iris_sami/status" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus Pause()
    {
        ros::NodeHandle n;
        boost::shared_ptr<const std_msgs::String> proceed = ros::topic::waitForMessage<std_msgs::String>("/iris_sami/continue", n);
        std::cout << proceed->data.c_str() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    class Sleep : public BT::AsyncActionNode
    {
    public:
        Sleep(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<float>("seconds") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<float> seconds = getInput<float>("seconds");
            if (!seconds)
            {
                throw BT::RuntimeError("missing required input [seconds]: ", seconds.error() );
            }

            sleep(seconds.value());
            return BT::NodeStatus::SUCCESS;
        }
    };

    class Velocity : public BT::AsyncActionNode
    {
    public:
        Velocity(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<float>("velocity") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<float> vel = getInput<float>("velocity");
            if (!vel)
            {
                throw BT::RuntimeError("missing required input [velocity]: ", vel.error() );
            }

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<iris_sami::Velocity>("iris_sami/velocity");
            iris_sami::Velocity srv;
            srv.request.velocity = vel.value();
            if (client.call(srv))
            {
                std::cout << srv.response.feedback << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "Failed to call service iris_sami/velocity" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    BT::NodeStatus grip()
    {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<iris_sami::NoArguments>("iris_sami/grip");
        iris_sami::NoArguments srv;
        if (client.call(srv))
        {
            std::cout << srv.response.feedback << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "Failed to call service iris_sami/grip" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus release()
    {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<iris_sami::NoArguments>("iris_sami/release");
        iris_sami::NoArguments srv;
        if (client.call(srv))
        {
            std::cout << srv.response.feedback << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "Failed to call service iris_sami/release" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    class Joints : public BT::AsyncActionNode
    {
    public:
        Joints(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<float>("shoulder_pan"),
                     BT::InputPort<float>("shoulder_lift"),
                     BT::InputPort<float>("elbow"),
                     BT::InputPort<float>("wrist_1"),
                     BT::InputPort<float>("wrist_2"),
                     BT::InputPort<float>("wrist_3") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<float> shoulder_pan = getInput<float>("shoulder_pan");
            BT::Optional<float> shoulder_lift = getInput<float>("shoulder_lift");
            BT::Optional<float> elbow = getInput<float>("elbow");
            BT::Optional<float> wrist_1 = getInput<float>("wrist_1");
            BT::Optional<float> wrist_2 = getInput<float>("wrist_2");
            BT::Optional<float> wrist_3 = getInput<float>("wrist_3");

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<iris_sami::JointGoal>("iris_sami/joints");
            iris_sami::JointGoal srv;

            srv.request.shoulder_pan = shoulder_pan.value();
            srv.request.shoulder_lift = shoulder_lift.value();
            srv.request.elbow = elbow.value();
            srv.request.wrist_1 = wrist_1.value();
            srv.request.wrist_2 = wrist_2.value();
            srv.request.wrist_3 = wrist_3.value();
            
            if (client.call(srv))
            {
                std::cout << srv.response.feedback << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "Failed to call service iris_sami/joints" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    class JointsAlias : public BT::AsyncActionNode
    {
    public:
        JointsAlias(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("alias") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<std::string> alias = getInput<std::string>("alias");

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<iris_sami::JointGoalName>("iris_sami/alias");
            iris_sami::JointGoalName srv;

            srv.request.name = alias.value();
            
            if (client.call(srv))
            {
                std::cout << srv.response.feedback << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "Failed to call service iris_sami/alias" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    class Pose : public BT::AsyncActionNode
    {
    public:
        Pose(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<float>("x"),
                     BT::InputPort<float>("y"),
                     BT::InputPort<float>("z"),
                     BT::InputPort<float>("roll"),
                     BT::InputPort<float>("pitch"),
                     BT::InputPort<float>("yaw") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<float> x = getInput<float>("x");
            BT::Optional<float> y = getInput<float>("y");
            BT::Optional<float> z = getInput<float>("z");
            BT::Optional<float> roll = getInput<float>("roll");
            BT::Optional<float> pitch = getInput<float>("pitch");
            BT::Optional<float> yaw = getInput<float>("yaw");

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<iris_sami::PoseGoal>("iris_sami/pose");
            iris_sami::PoseGoal srv;

            srv.request.x = x.value();
            srv.request.y = y.value();
            srv.request.z = z.value();
            srv.request.roll = roll.value();
            srv.request.pitch = pitch.value();
            srv.request.yaw = yaw.value();
            
            if (client.call(srv))
            {
                std::cout << srv.response.feedback << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "Failed to call service iris_sami/pose" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    class Move : public BT::AsyncActionNode
    {
    public:
        Move(const std::string& name, const BT::NodeConfiguration& config) 
        : AsyncActionNode(name, config){ }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<float>("x"),
                     BT::InputPort<float>("y"),
                     BT::InputPort<float>("z"),
                     BT::InputPort<float>("roll"),
                     BT::InputPort<float>("pitch"),
                     BT::InputPort<float>("yaw") };
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<float> x = getInput<float>("x");
            BT::Optional<float> y = getInput<float>("y");
            BT::Optional<float> z = getInput<float>("z");
            BT::Optional<float> roll = getInput<float>("roll");
            BT::Optional<float> pitch = getInput<float>("pitch");
            BT::Optional<float> yaw = getInput<float>("yaw");

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<iris_sami::RelativeMove>("iris_sami/move");
            iris_sami::RelativeMove srv;

            srv.request.x = x.value();
            srv.request.y = y.value();
            srv.request.z = z.value();
            srv.request.roll = roll.value();
            srv.request.pitch = pitch.value();
            srv.request.yaw = yaw.value();
            
            if (client.call(srv))
            {
                std::cout << srv.response.feedback << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "Failed to call service iris_sami/move" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    };
}