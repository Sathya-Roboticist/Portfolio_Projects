#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64

def twist_callback(msg):
    linear_x = msg.twist.twist.linear.x

    linear_x_pub.publish(linear_x)

if __name__ == '__main__':

    rospy.init_node('twist_linear_x_node')

    rospy.Subscriber('/neerakshi/dvl/twist', TwistWithCovarianceStamped, twist_callback)

    linear_x_pub = rospy.Publisher('/neerakshi/velocity', Float64, queue_size=10)

    rospy.spin()
