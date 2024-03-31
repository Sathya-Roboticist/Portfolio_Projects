#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_bht/auvAction.h>
#include <std_msgs/Float64.h> 

class auv_action_server {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<auv_bht::auvAction> as_;
    std::string action_name_;
    double thrust_;  
    ros::Publisher thruster_pub_;
    auv_bht::auvResult result_; 

public:
    auv_action_server(const std::string& name) :
        as_(nh_, name, boost::bind(&auv_action_server::executeCallback, this, _1), false),
        action_name_(name),
        thrust_(0.0) { 
        as_.start();
        thruster_pub_ = nh_.advertise<std_msgs::Float64>("thruster_topic", 1);  // Initialize the publisher
    }

   
    void executeCallback(const auv_bht::auvGoalConstPtr& goal) {
        ROS_INFO("Executing action: %s", action_name_.c_str());
        double period = 1.0;
        double thrust = goal->thrust;
        auv_bht::auvFeedback feedback;
        feedback.current_goal = goal->goal;  
        feedback.current_status = goal->status;

        int now = 0;
        while (now < 3) {
            if (!ros::ok()) {
                ROS_WARN("Action execution interrupted by ROS shutdown");
                return;
            }
            

            ros::Duration(period).sleep();
            now++;

            if (as_.isPreemptRequested()) {
                ROS_WARN("Action preempted");
                as_.setPreempted();
                return;
            }
        }

        // if (goal->status == true) {
        as_.publishFeedback(feedback);
        result_.result = true;
        ROS_INFO("Action succeeded");
        
        as_.setSucceeded(result_);
        // } else {
        //     ROS_WARN("Action aborted due to some condition");
        //     as_.setAborted(result_);
        // }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "check_server");
    auv_action_server server("neerakshi_in_action1");
    ros::spin();
    return 0;
}
