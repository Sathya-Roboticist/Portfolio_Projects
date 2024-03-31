#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_bht/auvAction.h> // Replace "your_package" and "YourAction" with your actual package and action names
#include <std_msgs/Bool.h>

class YourActionServer {
public:
    YourActionServer() : as_(nh_, "your_action_server", boost::bind(&YourActionServer::executeCallback, this, _1), false) {
        as_.start();
    }

    void executeCallback(const auv_bht::auvGoalConstPtr& goal) {
        ROS_INFO("Received goal: %d", goal->status);

        // Publish received goal as feedback
        
        
        auv_bht::auvFeedback feedback;
        feedback.current_status = goal->status;
        if(feedback.current_status){
            as_.publishFeedback(feedback);

            // Dummy processing, just to simulate some action
            ros::Duration(1).sleep();
        }

        // Send the result
        if(!feedback.current_status){
            auv_bht::auvResult result;
            result.success = goal->status;
            as_.setSucceeded(result);
        }
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<auv_bht::auvAction> as_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "your_action_server");
    YourActionServer server;
    ros::spin();
    return 0;
}
