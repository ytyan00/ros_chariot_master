#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class RGBRecorder:
    def __init__(self):
        rospy.init_node('rgb_recorder', anonymous=True)

        # Default save path to ~/catkin_ws/rgb_output.avi
        home_dir = os.path.expanduser('~')
        default_path = os.path.join(home_dir, 'catkin_ws', 'rgb_output.avi')

        self.output_path = rospy.get_param('~output_path', default_path)
        self.fps = rospy.get_param('~fps', 30.0)
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')

        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_size = None

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.loginfo(f"Recording RGB video from {self.image_topic} to {self.output_path}")
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.video_writer is None:
                self.frame_size = (cv_image.shape[1], cv_image.shape[0])
                self.video_writer = cv2.VideoWriter(
                    self.output_path,
                    cv2.VideoWriter_fourcc(*'XVID'),
                    self.fps,
                    self.frame_size
                )
                rospy.loginfo(f"Video writer initialized with size {self.frame_size}")

            self.video_writer.write(cv_image)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def __del__(self):
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo("Video writer released")

if __name__ == '__main__':
    try:
        RGBRecorder()
    except rospy.ROSInterruptException:
        pass
