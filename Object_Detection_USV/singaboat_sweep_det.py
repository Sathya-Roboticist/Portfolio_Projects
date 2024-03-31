#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode:
    def __init__(self, video_file_path):
        rospy.init_node('object_detection_node', anonymous=True)

        # Set the frame ID for the camera
        self.camera_frame_id = "wamv/front_left_camera_link"

        # OpenCV VideoCapture for reading from the MP4 file
        self.cap = cv2.VideoCapture(video_file_path)

        # Publisher for the annotated image
        self.annotated_image_pub = rospy.Publisher('/object_detection', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)
        
        # OpenCV bridge
        self.bridge = CvBridge()

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.loginfo("End of video file reached. Exiting...")
            rospy.signal_shutdown("End of video file reached.")
            return

        # Perform object detection (example: simple color-based detection)
        green_lower = np.array([40, 50, 50], dtype=np.uint8)
        green_upper = np.array([80, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(frame, green_lower, green_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area (you can adjust the minimum area as needed)
        min_contour_area = 5
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

        # Draw bounding boxes around detected large objects
        for contour in large_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Publish annotated image with the camera frame ID
        annotated_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        annotated_image_msg.header.frame_id = self.camera_frame_id
        self.annotated_image_pub.publish(annotated_image_msg)

    def run(self):
        try:
            rate = rospy.Rate(30)  # Assuming 30 frames per second
            while not rospy.is_shutdown():
                self.process_frame()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            # Release VideoCapture when done
            self.cap.release()

def main():
    try:
        # Replace 'your_video_file.mp4' with the actual path to your MP4 file
        video_file_path = '/home/oem/sweep_smart-2024-01-18_16.20.29.mp4'
        object_detection_node = ObjectDetectionNode(video_file_path)
        object_detection_node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
