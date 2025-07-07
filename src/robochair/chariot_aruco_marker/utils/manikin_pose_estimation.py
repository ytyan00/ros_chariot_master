#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PoseDetectorNode:
    def __init__(self):
        rospy.init_node("pose_detector", anonymous=True)

        # Initialize CvBridge for ROS <-> OpenCV conversions
        self.bridge = CvBridge()

        # Subscribe to RealSense RGB topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Initialize MediaPipe Pose Detector
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose( static_image_mode=False, model_complexity=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)

        # Initialize MediaPipe Drawing utilities
        self.mp_drawing = mp.solutions.drawing_utils

        rospy.loginfo("Pose Detector Node Initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert BGR to RGB (MediaPipe expects RGB format)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Process the image with MediaPipe Pose Detector
            results = self.pose.process(rgb_image)

            # Draw landmarks if detected
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

                # Extract landmark positions (example: Nose)
                nose = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE]
                rospy.loginfo(f"Nose Position - x: {nose.x:.2f}, y: {nose.y:.2f}, z: {nose.z:.2f}")

            # Display the processed image
            cv2.imshow("Pose Detection", cv_image)
            cv2.waitKey(1)  # Needed for OpenCV window updates

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == "__main__":
    try:
        node = PoseDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Detector Node Terminated.")
    finally:
        cv2.destroyAllWindows()
