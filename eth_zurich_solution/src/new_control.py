#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PControllerNode:
    def __init__(self):
        rospy.init_node('p_controller_node')
        self.kp_angular = rospy.get_param('~kp_angular', 1.5)
        self.kp_linear = rospy.get_param('~kp_linear', 0.1)  
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        filtered_ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))

        
        closest_obstacle = min(filtered_ranges)
        closest_obstacle_angle = scan_msg.angle_min + scan_msg.angle_increment * filtered_ranges.index(closest_obstacle)
        reference_angle = 0.0
        error = closest_obstacle_angle - reference_angle
        print("closest_obs_angle")
        print(closest_obstacle_angle)
        print("error")
        print(error)
        angular_vel = self.kp_angular * error

        distance_to_obstacle = closest_obstacle
        linear_vel = self.kp_linear * distance_to_obstacle

        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        twist_msg.linear.x = linear_vel
        self.cmd_pub.publish(twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PControllerNode()
    node.run()