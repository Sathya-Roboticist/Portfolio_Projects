#!/usr/bin/env python3

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

def callback(marker):
    listener = tf.TransformListener()
    try:
        listener.waitForTransform("odom", "rslidar", rospy.Time(), rospy.Duration(1.0))
        # marker = Marker()
        point_laser = PointStamped()
        point_laser.header.frame_id = "rslidar"
        point_laser.point = marker.pose.position   
        point_odom = listener.transformPoint("odom", point_laser)
        rospy.loginfo("Transformed point: %s", point_odom.point)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to transform point from laser frame to odom frame")

def listener():
    rospy.init_node('tf_listener')
    rospy.Subscriber('visualization_marker', Marker, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
