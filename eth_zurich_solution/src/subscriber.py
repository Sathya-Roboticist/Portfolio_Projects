#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import sys

def scanCallback(scan_msg):
    smallest_distance = sys.float_info.max
    # print("angle_min")
    # print(scan_msg.angle_min)
    # print("angle_max")
    # print(scan_msg.angle_max)
    # print("angle_increment")
    # print(scan_msg.angle_increment)

    for range in (scan_msg.ranges):
        if range < smallest_distance:
            smallest_distance = range
    #rospy.loginfo("Smallest distance: %f", smallest_distance)

def func():
    rospy.init_node('smb_highlevel_controller')
    topic_parameter = rospy.get_param("topic_name")
    q_s_parameter = rospy.get_param("queue_size")
    sub = rospy.Subscriber(topic_parameter, LaserScan, scanCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        func()
    except rospy.ROSInterruptException:
        pass