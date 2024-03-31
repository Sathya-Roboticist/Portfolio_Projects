#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <pugixml.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <auv_bht/auvAction.h>

using namespace BT;
// bool sa = false;
bool wait_for_continue_command = false;
bool actuator_status = false;

// class Path_Obstacle_Free : public SyncActionNode {
// public:
//     Path_Obstacle_Free(const std::string& name, const NodeConfiguration& config)
//         : SyncActionNode(name, config) {}

//     static PortsList providedPorts() {
//         return {InputPort<bool>("path_obstacle_free")};
//     }

//     NodeStatus tick() override {
//         // Read the desired RPM from the blackboard

//         bool path_obstacle_free;
//         if (!getInput<bool>("path_obstacle_free", path_obstacle_free)) {
//             std::cout<<"failure";
//             return NodeStatus::FAILURE;
//         }

//         // Create a goal message with the desired RPM
//         auv_bht::auvGoal id2;
//         id2.goal = 20.0;
//         id2.status = path_obstacle_free; // Use the provided rpm
//         // Send the goal to the ROS Action server
//         actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action1", true);

//         client.waitForServer();
//         client.sendGoal(id2);

//         // Wait for the result from the ROS Action server
//         if (client.waitForResult()) {
//             auto state = client.getState();
//             if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

//                 return NodeStatus::SUCCESS;
//             } else {
//                 return NodeStatus::FAILURE;
//             }
//         } else {
//             return NodeStatus::FAILURE;
//         }
//     }
// };

class Mission_Planner_Abort : public SyncActionNode {
public:
    Mission_Planner_Abort(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("mission_abort_status")};
    }

    NodeStatus tick() override {
        ros::NodeHandle nh;

        bool mission_abort_status;



        if (!getInput<bool>("mission_abort_status", mission_abort_status)) {
            std::cout<<"failure";
            return NodeStatus::FAILURE;
        }
        // Read the desired RPM from the blackboard
        // std::cout<<sa;
        auv_bht::auvGoal id;
        id.goal = 10.0;
        id.status = mission_abort_status;
        
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                std::cout<<"i am here1!";
                return NodeStatus::SUCCESS;
            } else {
                std::cout<<"i am failed!";
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};

class Wait_For_Continue_Command : public SyncActionNode {
public:
    Wait_For_Continue_Command(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        // bool wait_for_continue_command;
        // if (!getInput<bool>("wait_for_continue_command", wait_for_continue_command)) {
        //     std::cout<<"fail";
        //     return NodeStatus::FAILURE;
        // }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id3;
        id3.goal = 30.0;
        id3.status = wait_for_continue_command;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id3);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};

// class Continue_Command_Received : public SyncActionNode {
// public:
//     Continue_Command_Received(const std::string& name, const NodeConfiguration& config)
//         : SyncActionNode(name, config) {}

//     static PortsList providedPorts() {
//         return {};
//     }

//     NodeStatus tick() override {
//         // Read the desired RPM from the blackboard

//         bool continue_command_recieved;
//         // if (!getInput<bool>("continue_command_recieved", continue_command_recieved)) {
//         //     std::cout<<"failure14";
//         //     return NodeStatus::FAILURE;
//         // }

//         // Create a goal message with the desired RPM
//         // std::cout<<"i am here";
//         auv_bht::auvGoal id4;
//         id4.goal = 40.0;
//         id4.status = continue_command_recieved;// Use the provided rpm
//         // Send the goal to the ROS Action server
//         actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

//         client.waitForServer();
//         client.sendGoal(id4);

//         // Wait for the result from the ROS Action server
//         if (client.waitForResult()) {
//             auto state = client.getState();
//             if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
//                 return NodeStatus::SUCCESS;
//             } else {
//                 return NodeStatus::FAILURE;
//             }
//         } else {
//             return NodeStatus::FAILURE;
//         }
//     }
// };

// class Continue_Command_Received : public BT::ConditionNode {
// public:
//     Continue_Command_Received(const std::string& name, const BT::NodeConfiguration& config)
//         : BT::ConditionNode(name, config), continue_command_received_(false) {
//             // Initialize ROS subscriber
//             nh_ = ros::NodeHandle();
//             sub_ = nh_.subscribe("Mission_Planner_Abort", 1, &Continue_Command_Received::boolCallback, this);
//     }
//     static BT::PortsList providedPorts() {
//     return {};
//     }

//     BT::NodeStatus tick() override {
//         if (!continue_command_received_) {
//             return BT::NodeStatus::RUNNING; // Stay in running state until message is received
//         } 
//         else {
//             auv_bht::auvGoal goal;
//             goal.goal = 40.0; // Set desired RPM
//             goal.status = continue_command_received_;

//             // Send the goal to the ROS Action server
//             actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);
//             client.waitForServer();
//             client.sendGoal(goal);
//             return BT::NodeStatus::SUCCESS; // Succeed once message is received
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_;
//     bool continue_command_received_;

//     void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
//         // Callback function to receive boolean data from the topic
//         continue_command_received_ = msg->data;
//     }
// };


class Continue_Command_Received : public BT::SyncActionNode {
public:
    Continue_Command_Received(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), continue_command_received_(false) {
            // Initialize ROS subscriber
            nh_ = ros::NodeHandle();
            sub_ = nh_.subscribe("Mission_Planner_Abort", 1, &Continue_Command_Received::boolCallback, this);
    }

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        // Check if continue command has been received
        if (!continue_command_received_) {
            return BT::NodeStatus::FAILURE; // Stay in running state until message is received
        }

        // Create a goal message with the received continue command status
        auv_bht::auvGoal goal;
        goal.goal = 40.0; // Set desired RPM
        goal.status = continue_command_received_;

        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);
        client.waitForServer();
        client.sendGoal(goal);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool continue_command_received_;

    void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
        // Callback function to receive boolean data from the topic
        continue_command_received_ = msg->data;
    }
};


class Actuator_Status : public SyncActionNode {
public:
    Actuator_Status(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), actuator_status_received_(false) {
            // Initialize the ROS subscriber
            nh_ = ros::NodeHandle();
            sub_ = nh_.subscribe("Mission_Planner_Abort", 1, &Actuator_Status::messageCallback, this);
        }

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override {
        // Check if actuator status has been received
        if (!actuator_status_received_) {

            return NodeStatus::FAILURE; // Return FAILURE until message is received
        }

        // Create a goal message with the received actuator status
        auv_bht::auvGoal id4;
        id4.goal = 20.0;
        id4.status = actuator_status_;

        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action1", true);
        client.waitForServer();
        client.sendGoal(id4);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool actuator_status_received_;
    bool actuator_status_;

    void messageCallback(const std_msgs::Bool::ConstPtr& msg) {
        actuator_status_received_ = true;
        actuator_status_ = msg->data;
    }
};




class Leak_Status : public SyncActionNode {
public:
    Leak_Status(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("leak_status")};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        bool leak_status;
        if (!getInput<bool>("leak_status", leak_status)) {
            std::cout<<"failure16";
            return NodeStatus::FAILURE;
        }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id5;
        id5.goal = 30.0;
        id5.status = leak_status;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id5);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};

class Set_Mission_Abort : public SyncActionNode {
public:
    Set_Mission_Abort(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("set_mission_abort")};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        bool set_mission_abort;
        if (!getInput<bool>("set_mission_abort", set_mission_abort)) {
            std::cout<<"failure17";
            return NodeStatus::FAILURE;
        }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id11;
        id11.goal = 50.0;
        id11.status = set_mission_abort;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id11);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};

class Go_To_Surface : public SyncActionNode {
public:
    Go_To_Surface(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("go_to_surface")};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        bool go_to_surface;
        if (!getInput<bool>("go_to_surface", go_to_surface)) {
            std::cout<<"failure11";
            return NodeStatus::FAILURE;
        }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id4;
        id4.goal = 50.0;
        id4.status = go_to_surface;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id4);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};

class Path_Obstacle_Free : public SyncActionNode {
public:
    Path_Obstacle_Free(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("path_obstacle_free")};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        bool path_obstacle_free;
        if (!getInput<bool>("path_obstacle_free", path_obstacle_free)) {
            std::cout<<"failure12";
            return NodeStatus::FAILURE;
        }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id4;
        id4.goal = 60.0;
        id4.status = path_obstacle_free;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id4);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};


class Avoid_Obstacles : public SyncActionNode {
public:
    Avoid_Obstacles(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<bool>("avoid_obstacles")};
    }

    NodeStatus tick() override {
        // Read the desired RPM from the blackboard

        bool avoid_obstacles;
        if (!getInput<bool>("avoid_obstacles", avoid_obstacles)) {
            std::cout<<"failure13";
            return NodeStatus::FAILURE;
        }

        // Create a goal message with the desired RPM
        // std::cout<<"i am here";
        auv_bht::auvGoal id4;
        id4.goal = 70.0;
        id4.status = avoid_obstacles;// Use the provided rpm
        // Send the goal to the ROS Action server
        actionlib::SimpleActionClient<auv_bht::auvAction> client("neerakshi_in_action", true);

        client.waitForServer();
        client.sendGoal(id4);

        // Wait for the result from the ROS Action server
        if (client.waitForResult()) {
            auto state = client.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::FAILURE;
        }
    }
};





void messageCallback(const std_msgs::Bool::ConstPtr& msg) {
    actuator_status = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "auv_bt_client_server");
    // ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe<std_msgs::Bool>("Mission_Planner_Abort", 1000, messageCallback);
    // std::cout<<sa;
    BehaviorTreeFactory factory;
    factory.registerNodeType<Mission_Planner_Abort>("Mission_Planner_Abort");
    // factory.registerNodeType<Path_Obstacle_Free>("Path_Obstacle_Free");
    // factory.registerNodeType<Wait_For_Continue_Command>("Wait_For_Continue_Command");
    factory.registerNodeType<Continue_Command_Received>("Continue_Command_Received");
    // factory.registerNodeType<Actuator_Status>("Actuator_Status");
    factory.registerNodeType<Leak_Status>("Leak_Status");
    factory.registerNodeType<Set_Mission_Abort>("Set_Mission_Abort");
    factory.registerNodeType<Go_To_Surface>("Go_To_Surface");
    factory.registerNodeType<Path_Obstacle_Free>("Path_Obstacle_Free");
    factory.registerNodeType<Avoid_Obstacles>("Avoid_Obstacles");
    
    std::string xml = "/home/oem/pers_ws/src/auv_bht/tree/auv_bh_fd.xml";
    auto tree = factory.createTreeFromFile(xml);
    auto zmq_publisher = std::make_unique < PublisherZMQ > (tree);
    printTreeRecursively(tree.rootNode());

    NodeStatus status = NodeStatus::RUNNING;
    while (ros::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickRootWhileRunning();
        ros::spinOnce();
        ros::Duration(0.1).sleep(); // Adjust the sleep duration as needed
    }

    return 0;
}
