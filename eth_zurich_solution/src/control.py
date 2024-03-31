#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PControllerNode:
    def __init__(self):
        rospy.init_node('p_controller_node')
        self.kp = rospy.get_param('~kp', 0.5)  
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def scan_callback(self, scan_msg):
        filtered_ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))


        min_angle = scan_msg.angle_min
        print("minimum angle")
        print(min_angle)
        print("angle_inc")
        
        angle_increment = scan_msg.angle_increment
        print(angle_increment)
        # val = filtered_ranges.index(min(filtered_ranges))
        # val = min(filtered_ranges)
        # print(val)
        closest_obstacle_angle = min_angle + angle_increment * filtered_ranges.index(min(filtered_ranges))
        # print(closest_obstacle_angle)
        reference_angle = 0.0 
        error = closest_obstacle_angle - reference_angle
        angular_vel = self.kp * error
        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        # self.cmd_pub.publish(twist_msg)
        twist_msg.linear.x = 1.0
        self.cmd_pub.publish(twist_msg)
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PControllerNode()
    node.run()
