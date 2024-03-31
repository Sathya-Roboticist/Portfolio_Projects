#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from mvp_msgs.msg import Float64Stamped

class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter')
        self.depth_pub = rospy.Publisher('/neerakshi/dvl_link_altitude_modified', Float64, queue_size=10)
        rospy.Subscriber('/neerakshi/depth', Float64Stamped, self.depth_callback)
        self.initial_depth = None  # Initialize the initial depth variable

    def depth_callback(self, data):
        # if self.initial_depth is None:
        #     self.initial_depth = data.range  # Set the initial depth only once

        current_depth = data.data

        # Adjusting the range values dynamically
        # modified_depth = max(0.0, self.initial_depth - current_depth)

        self.depth_pub.publish(current_depth)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        depth_converter = DepthConverter()
        depth_converter.run()
    except rospy.ROSInterruptException:
        pass
