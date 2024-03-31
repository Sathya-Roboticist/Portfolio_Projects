#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry

def callback(data):
    # Process the received message
    # ...
    #print(3)
    print(data)
    rospy.sleep(0.5)  # Add a 1-second delay

def listener():
    rospy.init_node('message_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()