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
        
        # Initialize camera_info_msg
        self.camera_info_msg = CameraInfo()

    def camera_info_sub_callback(self, msg):
        try:
            # Extract camera info parameters from the received message
            self.camera_info_msg.header = msg.header
            self.camera_info_msg.width = msg.width
            self.camera_info_msg.height = msg.height
            self.camera_info_msg.distortion_model = msg.distortion_model
            self.camera_info_msg.D = msg.D
            self.camera_info_msg.K = msg.K
            self.camera_info_msg.R = msg.R
            self.camera_info_msg.P = msg.P
            self.camera_info_msg.binning_x = msg.binning_x
            self.camera_info_msg.binning_y = msg.binning_y
            self.camera_info_msg.roi = msg.roi
            
            # Set the frame ID for the camera info message
            self.camera_info_msg.header.frame_id = self.camera_frame_id
            
            # Publish camera info continuously
            self.camera_info_pub.publish(self.camera_info_msg)

        except Exception as e:
            rospy.logerr("Error processing CameraInfo message: {}".format(e))


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr("Error converting image: {}".format(e))
            return

        # Convert the image to HSV for better color-based segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of blue color in HSV
        blue_lower = np.array([100, 50, 50], dtype=np.uint8)
        blue_upper = np.array([130, 255, 255], dtype=np.uint8)

        # Create a mask using the inRange function to filter out only blue color
        mask = cv2.inRange(hsv_image, blue_lower, blue_upper)

        # Apply morphological operations to improve object connectivity
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)

        # Find contours and filter based on circularity and contour area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_contour_area = 100  # Adjust as needed

        for contour in contours:
            # Calculate circularity using the formula: (4 * pi * area) / (perimeter^2)
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Add a check to prevent division by zero
            if perimeter > 0:
                circularity = (4 * np.pi * area) / (perimeter**2)

                # Adjust circularity threshold based on your requirements
                circularity_threshold = 0.5

                if area > min_contour_area and circularity > circularity_threshold:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Use the header from the camera info message instead of the original image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        annotated_image_msg.header = self.camera_info_msg.header
        self.annotated_image_pub.publish(annotated_image_msg)

def main():
    try:
        object_detection_node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
