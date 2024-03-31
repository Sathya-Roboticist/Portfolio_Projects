#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

def imu_callback(msg):
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = euler_from_quaternion(quaternion)

    euler_msg = Vector3()
    euler_msg.x = euler[0]  
    euler_msg.y = euler[1]  
    euler_msg.z = euler[2]  

    orientation_pub.publish(euler_msg)

if __name__ == '__main__':
    rospy.init_node('orientation_converter')

    imu_sub = rospy.Subscriber('/neerakshi/imu/stonefish/data', Imu, imu_callback)

    orientation_pub = rospy.Publisher('/auv3/orientation', Vector3, queue_size=10)

    rospy.spin()
