#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf
import tf2_geometry_msgs  # for PoseStamped transform()

def rotation_vector_to_euler(rvec):
    """Convert rotation vector to Euler angles in degrees."""
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])

class ArucoPoseEstimator:
    def __init__(self):
        rospy.init_node("aruco_pose_estimator", anonymous=True)
        self.bridge = CvBridge()

        # image subscriber
        self.image_sub = rospy.Subscriber(
            "/kinova_wrist/color/image_raw", Image, self.image_callback, queue_size=1
        )

        # publish marker pose
        self.pose_pub = rospy.Publisher(
            "aruco_marker_pose", PoseStamped, queue_size=10
        )
        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ArUco setup
        self.aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_size = 0.025  # meters

        # camera intrinsics
        # — 1. Wait for exactly one CameraInfo message —
        info = rospy.wait_for_message("/kinova_wrist/color/camera_info", CameraInfo)
        K = info.K
        self.camera_matrix = np.array(K, dtype=np.float32).reshape((3,3))
        self.dist_coeffs  = np.array(info.D, dtype=np.float32)
        print("Camera intrinsics loaded from /kinova_wrist/color/camera_info")


    def image_callback(self, msg):
        try:
            # convert and grayscale
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is None:
                cv2.imshow("Aruco", frame)
                cv2.waitKey(1)
                return

            # estimate pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size,
                self.camera_matrix, self.dist_coeffs
            )

            for i, mid in enumerate(ids.flatten()):
                # draw marker + axes
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix, self.dist_coeffs,
                    rvecs[i], tvecs[i],
                    0.05, 2
                )

                # Build PoseStamped in camera_color_optical_frame

                # cv2 camera frame & camera_color_optical_frame
                # +X points to the right in the image
                # +Y points down in the image
                # +Z points forward, out of the lens
                
                # RGB camera is 0.05m from camera center
                # This offset is taken care by realsense2_camera


                p = PoseStamped()
                p.header.frame_id = "kinova_wrist_color_optical_frame"
                p.header.stamp = msg.header.stamp
                p.pose.position.x = float(tvecs[i][0,0])
                p.pose.position.y = float(tvecs[i][0,1])
                p.pose.position.z = float(tvecs[i][0,2])

                # rotation → quaternion
                F = np.diag([-1, -1, 1])   # flips X and Y axes
                # rvecs[i] is axis-angle in CV frame
                R_cv, _   = cv2.Rodrigues(rvecs[i])       # 3×3 in CV frame
                R_urdf    = F @ R_cv @ F                  # flip into URDF frame

                M         = np.eye(4)
                M[:3,:3]  = R_urdf
                q         = tf.transformations.quaternion_from_matrix(M)
                p.pose.orientation.x = q[0]
                p.pose.orientation.y = q[1]
                p.pose.orientation.z = q[2]
                p.pose.orientation.w = q[3]

                self.pose_pub.publish(p)
                print(
                f"[Marker {mid}] pos(d435i): "
                f"[{p.pose.position.x:.3f}, "
                f"{p.pose.position.y:.3f}, "
                f"{p.pose.position.z:.3f}] "
                )

            # show
            cv2.imshow("Aruco", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ArucoPoseEstimator().run()
