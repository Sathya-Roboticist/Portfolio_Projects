#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker():
    rospy.init_node('marker_publisher')
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "rslidar"  
        marker.header.stamp = rospy.Time.now()  
        marker.type 
        marker.ns = "my_namespace"  
        marker.id = 0  
        marker.type = Marker.ARROW  
        marker.action = Marker.ADD  
        marker.pose.orientation.w = 1.0  
        marker.scale.x = 0.1  
        marker.scale.y = 0.2  
        marker.scale.z = 0.3  
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  

        
        base_point = Point()
        base_point.x = 20.000925
        base_point.y = 4.997748
        base_point.z = 5.0
        

        tip_point = Point()
        tip_point.x = 20.000925
        tip_point.y = 4.997748
        tip_point.z = 2.499991

        marker.points.append(base_point)
        marker.points.append(tip_point)
        marker.lifetime = rospy.Duration() 
        marker_pub.publish(marker)  

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
