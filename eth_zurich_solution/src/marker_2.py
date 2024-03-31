#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

def publish_marker():

    rospy.init_node('marker_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "rslidar"  
        marker.type = Marker.SPHERE  
        marker.action = Marker.ADD  
        marker.pose.position.x = 20.000925 
        marker.pose.position.y = 4.997748
        marker.pose.position.z = 2.499991
        marker.pose.orientation.w = 1.0  

        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        
        marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass