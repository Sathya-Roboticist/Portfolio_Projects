#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

pub = rospy.Publisher("/pcl_publisher", PointCloud2, queue_size=10)

def pcl_callback(msg : PointCloud2):
    pcl = msg.height * msg.row_step
    rospy.loginfo_throttle(2.0, pcl)
    # pub.publish(pcl)

def func():
    rospy.init_node('pcl_node', anonymous=True)
    sub =rospy.Subscriber("/rslidar_points", PointCloud2, pcl_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        func()
    except rospy.ROSInterruptException():
        pass

