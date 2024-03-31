#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)

        # Set the frame ID for the camera
        self.camera_frame_id = "wamv/front_left_camera_link"

        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber('/wamv/sensors/cameras/front_left_camera/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/wamv/sensors/cameras/front_left_camera/camera_info', CameraInfo, self.camera_info_sub_callback)
        
        # Publisher for the annotated image
        self.annotated_image_pub = rospy.Publisher('/object_detection', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)
        
        # OpenCV bridge
        self.bridge = CvBridge()

    def camera_info_sub_callback(self, msg):
        try:
            # Extract camera info parameters from the received message
            camera_info_msg = CameraInfo()
            camera_info_msg.header = msg.header
            camera_info_msg.width = msg.width
            camera_info_msg.height = msg.height
            camera_info_msg.distortion_model = msg.distortion_model
            camera_info_msg.D = msg.D
            camera_info_msg.K = msg.K
            camera_info_msg.R = msg.R
            camera_info_msg.P = msg.P
            camera_info_msg.binning_x = msg.binning_x
            camera_info_msg.binning_y = msg.binning_y
            camera_info_msg.roi = msg.roi
            
            # Set the frame ID for the camera info message
            camera_info_msg.header.frame_id = self.camera_frame_id
            
            # Publish camera info continuously
            self.camera_info_pub.publish(camera_info_msg)

        except Exception as e:
            rospy.logerr("Error processing CameraInfo message: {}".format(e))


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr("Error converting image: {}".format(e))
            return

        # Adjust color-based detection parameters
        green_lower = np.array([40, 50, 50], dtype=np.uint8)
        green_upper = np.array([80, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(cv_image, green_lower, green_upper)

        # Apply morphological operations to improve object connectivity
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)

        # Find contours and filter based on contour area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_contour_area = 100  # Adjust as needed

        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        annotated_image_msg.header.frame_id = self.camera_frame_id
        self.annotated_image_pub.publish(annotated_image_msg)

def main():
    try:
        object_detection_node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
