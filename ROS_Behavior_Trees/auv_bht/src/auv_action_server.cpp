#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_bht/auvAction.h>
#include <std_msgs/Float64.h>  // Include the Float64 message type for publishing thrust value

class auv_action_server {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<auv_bht::auvAction> as_;
    std::string action_name_;
    double thrust_;  // Variable to hold the thrust value
    ros::Publisher thruster_pub_;  // Publisher for the thruster topic
    auv_bht::auvResult result_; 

public:
    auv_action_server(const std::string& name) :
        as_(nh_, name, boost::bind(&auv_action_server::executeCallback, this, _1), false),
        action_name_(name),
        thrust_(0.0) {  // Initialize thrust_ to zero
        as_.start();
        thruster_pub_ = nh_.advertise<std_msgs::Float64>("thruster_topic", 1);  // Initialize the publisher
    }

    // Callback function to execute when a goal is received
    void executeCallback(const auv_bht::auvGoalConstPtr& goal) {
        ROS_INFO("Executing action: %s", action_name_.c_str());
        double period = 1.0;
        double thrust = goal->thrust;
        auv_bht::auvFeedback feedback;
        feedback.current_goal = goal->goal;  // Initialize feedback
        feedback.current_status = goal->status;

        // Execute the action logic
        int now = 0;
        while (now < 2) {
            if (!ros::ok()) {
                ROS_WARN("Action execution interrupted by ROS shutdown");
                return;
            }
            // Publish feedback
            

            // Sleep for the defined period
            ros::Duration(period).sleep();
            now++;

            // Check if the action should be preempted
            if (as_.isPreemptRequested()) {
                ROS_WARN("Action preempted");
                as_.setPreempted();
                return;
            }
        }

        // Check if the goal status should be set to succeeded or aborted
        if (goal->status == true) {
            result_.result = true;
            ROS_INFO("Action succeeded");
            as_.publishFeedback(feedback);
            as_.setSucceeded(result_);
        } else {
            ROS_WARN("Action aborted due to some condition");
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auv_action_server");
    auv_action_server server("neerakshi_in_action");
    ros::spin();
    return 0;
}
