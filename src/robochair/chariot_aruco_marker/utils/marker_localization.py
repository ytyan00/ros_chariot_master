#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class ArucoLocator:
    def __init__(self):
        rospy.init_node("aruco_locator", anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribe to RealSense RGB topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # ArUco dictionary and parameters
        self.aruco_dict = self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)  # ✅ Works in OpenCV 4.11.0

        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Camera intrinsic parameters (Replace with real calibration values)
        self.camera_matrix = np.array([[635.9087524414062, 0.0, 654.427001953125], [0.0, 635.2030029296875, 361.94482421875], [0, 0, 1]], dtype=np.float32)  # Fx, Fy, Cx, Cy
        self.dist_coeffs = np.array([-0.060172442346811295, 0.07232918590307236, -0.0005943544092588127, -0.0006938826991245151, -0.023473644629120827])  # Assuming no distortion for simplicity

        # Known reference marker (Set this ID manually or dynamically)
        self.reference_marker_id = 1
        self.known_marker_position = np.array([0, 0, 0])  # Replace with real-world coordinates

         # **Marker Size in meters (15cm = 0.15m)**
        self.marker_size = 0.15  # 15 cm marker

        rospy.loginfo("Aruco locator initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                # Estimate pose of each detected marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                # Store marker positions
                marker_positions = {}

                for i, marker_id in enumerate(ids.flatten()):
                    position = tvecs[i].flatten()
                    marker_positions[marker_id] = position

                    # Draw markers
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    cv2.putText(cv_image, f"ID: {marker_id} Pos: {position.round(2)}",
                                tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Compute relative positions based on known reference marker
                if self.reference_marker_id in marker_positions:
                    ref_position = marker_positions[self.reference_marker_id]
                    for marker_id, position in marker_positions.items():
                        if marker_id != self.reference_marker_id:
                            relative_position = position - ref_position
                            rospy.loginfo(f"Marker {marker_id} relative to {self.reference_marker_id}: {relative_position.round(2)}")

            # Display image
            cv2.imshow("Aruco Locator", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        locator = ArucoLocator()
        locator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco locator node terminated.")
    finally:
        cv2.destroyAllWindows()
