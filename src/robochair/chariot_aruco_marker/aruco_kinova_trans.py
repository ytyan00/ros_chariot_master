#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class ArucoBaseTransformer:
    def __init__(self, verbose = False):
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber to the marker pose in camera frame
        self.sub = rospy.Subscriber(
            '/aruco_marker_pose', PoseStamped,
            self.pose_callback, queue_size=10
        )

        # log
        self.verbose = verbose
        self.pose = None
        
    def pose_callback(self, msg: PoseStamped):
        try:
            # Wait for transform from camera frame to base_link
            target_frame = 'base_link'
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(1.0)
            )

            # Transform PoseStamped into base_link frame
            pose_base = tf2_geometry_msgs.do_transform_pose(msg, transform)

            # Print marker position
            pos = pose_base.pose.position
            if self.verbose:
                print(
                    f"[Aruco] Marker in {target_frame}: "
                    f"x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}"
                )
            self.pose = pose_base
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 transform failed: {e}")
        
    def get_filter_pos(self):
        pass
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('aruco_to_base_link', anonymous=True)
    node = ArucoBaseTransformer(verbose=True)
    node.run()
