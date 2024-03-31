#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <chrono> 

int main(int argc, char **argv) {
    ros::init(argc, argv, "publishers");
    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("Mission_Planner_Abort", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("Path_Obstacle_Free", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::Bool>("Wait_For_Continue_Command", 10);

    ros::Rate rate(1);
    auto start_time = std::chrono::steady_clock::now();

    while (ros::ok()) {
        auto current_time = std::chrono::steady_clock::now() - start_time;
        double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time).count();

        std_msgs::Bool msg;
        
        if (elapsed_time < 5.0) {
            msg.data = true;
            pub1.publish(msg);
            msg.data = true;
            pub2.publish(msg);
            pub3.publish(msg);
        } else if (elapsed_time < 10.0) {
            msg.data = true;
            pub1.publish(msg);
            pub2.publish(msg);
            msg.data = true;
            pub3.publish(msg);
        } else if (elapsed_time < 15.0) {
            msg.data = true;
            pub1.publish(msg);
            msg.data = true;
            pub2.publish(msg);
            pub3.publish(msg);
        } else {
            start_time = std::chrono::steady_clock::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
