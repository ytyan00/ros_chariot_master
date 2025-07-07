#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
# from pick_point_helper import get_distance
from pick_point_helper import get_distance

import threading
import ros_numpy
import numpy as np
import open3d as o3d
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge



class camera_position_publisher():
    def __init__(self) -> None:
        rospy.init_node('camera_listener_point_publisher_node')
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.depth_intrinsics = None
        # subscriber
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        # publisher (Create a publisher for the "target" topic, publishing Float32 messages)
        self.target_publisher = rospy.Publisher('/target', Point, queue_size=1)
        # other vars
        self.rate = rospy.Rate(5)  # Set the publishing rate (1 Hz in this example)
        self.value = Point()
        self.update = False

    # ----- callback functions -----
    def depth_callback(self,msg):
        self.depth_image = ros_numpy.numpify(msg)

    def color_callback(self,msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_info_callback(self,msg):
        self.depth_intrinsics = msg
    

    def update_value(self):
        while not rospy.is_shutdown() and self.update == False:
            # getting a new value (e.g., from a sensor or calculation)
            x, y,z = get_distance(self.depth_image,self.color_image,self.depth_intrinsics)
            # x ,y position in camera coordinate, we assume that camera coordinate 
            # is the same as world coordinate
            self.value = Point(x- 0.2, y - 0.2 , z - 0.2)
            self.update = True

    def run(self):
        while not rospy.is_shutdown():
            # Publish the value to the "target" topic
            self.target_publisher.publish(self.value)
            print("[INFO] Published")
            self.rate.sleep()

if __name__ == '__main__':
    set_target = camera_position_publisher()
    set_target.run()
