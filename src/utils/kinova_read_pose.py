import sys
import os
import rospy
import time
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse
import numpy as np

kinova_folder = "/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl"
sys.path.append(kinova_folder)
from kinova import KinovaArm
from kinova_get_yaml_pose import get_yaml_poses


aruco_folder = "/home/chariot/catkin_ws/src/robochair/src/chariot_aruco_marker"
sys.path.append(aruco_folder)
from aruco_kinova_trans import ArucoBaseTransformer


robot = KinovaArm()

while True:
    q,ee_pose,_ = robot.get_state()
    print(ee_pose)
