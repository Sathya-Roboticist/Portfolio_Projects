#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <pugixml.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>


using namespace BT;

class Node1Child1Condition : public BT::ConditionNode {
public:
    Node1Child1Condition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override {
        if (data_received_ == false) {
            data_received_ = false;
            ROS_INFO("I am here");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    void topicCallback(const std_msgs::Bool::ConstPtr& msg) {
        data_received_ = msg->data;
        
    }

private:
    ros::Subscriber sub_;
    bool data_received_;
    std::string cmp = "True"; 
};


class Node1Child2Condition : public BT::ConditionNode {
public:
    Node1Child2Condition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), data_received_(false) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override {
        if (data_received_ == false) {
            data_received_ = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    void topicCallback(const std_msgs::Bool::ConstPtr& msg) {
        data_received_ = msg->data;
    }


private:
    ros::Subscriber sub_;
    bool data_received_;
};

class Node2ChildCondition : public BT::ConditionNode {
public:
    Node2ChildCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), data1_received_(false), data2_received_(false) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override {
        if (data1_received_ && data2_received_ == false) {
            data1_received_ = false;
            data2_received_ = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    void topic1Callback(const std_msgs::Bool::ConstPtr& msg) {
        data1_received_ = msg->data;
    }

    void topic2Callback(const std_msgs::Bool::ConstPtr& msg) {
        data2_received_ = msg->data;
    }

private:
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    bool data1_received_;
    bool data2_received_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bt_ros_example");
    ros::NodeHandle nh;

    BehaviorTreeFactory factory;

    factory.registerNodeType<Node1Child1Condition>("Node1Child1");
    factory.registerNodeType<Node1Child2Condition>("Node1Child2");
    factory.registerNodeType<Node2ChildCondition>("Node2Child");

    const std::string xml_text = R"(
    <root main_tree_to_execute="main_tree">
        <BehaviorTree ID="main_tree">
            <Parallel success_threshold="2" failure_threshold="0">
                <Sequence>
                    <Node1Child1 />
                    <Node1Child2 />
                </Sequence>
                <Sequence>
                    <Node2Child />
                </Sequence>
            </Parallel>
        </BehaviorTree>
    </root>

    )";

    auto tree = factory.createTreeFromText(xml_text);
    auto zmq_publisher = std::make_unique < PublisherZMQ > (tree);


    StdCoutLogger logger_cout(tree);

    Node1Child1Condition node1Child1Condition("Node1Child1", BT::NodeConfiguration());
    Node1Child2Condition node1Child2Condition("Node1Child2", BT::NodeConfiguration());
    Node2ChildCondition node2ChildCondition("Node2Child", BT::NodeConfiguration());

    ros::Subscriber sub1 = nh.subscribe<std_msgs::Bool>("Mission_Planner_Abort", 1,
        boost::bind(&Node1Child1Condition::topicCallback, &node1Child1Condition, _1));
    ros::Subscriber sub2 = nh.subscribe<std_msgs::Bool>("Mission_Planner_Abort", 1,
        boost::bind(&Node1Child2Condition::topicCallback, &node1Child2Condition, _1));
    ros::Subscriber sub3 = nh.subscribe<std_msgs::Bool>("Path_Obstacle_Free", 1,
        boost::bind(&Node2ChildCondition::topic1Callback, &node2ChildCondition, _1));
    ros::Subscriber sub4 = nh.subscribe<std_msgs::Bool>("Wait_For_Continue_Command", 1,
        boost::bind(&Node2ChildCondition::topic2Callback, &node2ChildCondition, _1));

    while (ros::ok()) {
        tree.tickRoot();
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(3)); // Check every second
    }

    return 0;
}

