#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <auv_bht/auvAction.h> 
#include <std_msgs/Bool.h>

class YourActionClient {
public:
    YourActionClient() : ac_("your_action_server", true) { // Replace "your_action_server" with the name of your action server
        sub_ = nh_.subscribe("Mission_Planner_Abort", 1, &YourActionClient::boolCallback, this); // Subscribe to the topic publishing boolean data
        ac_.waitForServer();
    }

    void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
        // Callback function to receive boolean data from the topic
        auv_bht::auvGoal goal;
        goal.status = msg->data;

        ac_.sendGoal(goal, boost::bind(&YourActionClient::doneCallback, this, _1, _2),
                     boost::bind(&YourActionClient::activeCallback, this),
                     boost::bind(&YourActionClient::feedbackCallback, this, _1));
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const auv_bht::auvResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Result: %d", result->success);
        }
    }

    void activeCallback() {
        ROS_INFO("Goal just went active");
    }

    void feedbackCallback(const auv_bht::auvFeedbackConstPtr& feedback) {
        ROS_INFO("Feedback: %d", feedback->current_status);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    actionlib::SimpleActionClient<auv_bht::auvAction> ac_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "your_action_client");
    YourActionClient client;
    ros::spin();
    return 0;
}
