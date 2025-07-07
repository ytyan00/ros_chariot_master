#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("aruco_detector", anonymous=True)

        # Create a CvBridge instance
        self.bridge = CvBridge()

        # Subscribe to RealSense RGB camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Define the ArUco dictionary (5x5 markers)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()

        rospy.loginfo("Aruco detector initialized. Waiting for images...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                # Draw detected markers
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                rospy.loginfo(f"Detected ArUco Markers: {ids.flatten()}")

            # Display the image
            cv2.imshow("Aruco Marker Detection", cv_image)
            cv2.waitKey(1)  # Needed for OpenCV window to update

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == "__main__":
    try:
        detector = ArucoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco detector node terminated.")
    finally:
        cv2.destroyAllWindows()
