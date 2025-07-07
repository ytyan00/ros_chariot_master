#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import sys
import os
import time
# Add the directory to sys.path
KINOVA_DIR = '/home/yunting/untie_diffusion/data_collection_and_kinova_controller/catkin_ws/cs6751/src/kinova_controller'
sys.path.append(KINOVA_DIR)

# Now import the module like a normal Python file in that folder
from kinova import KinovaArm
from kinova_get_yaml_pose import get_yaml_poses



class GraspStrap:
    def __init__(self):
        self.arm = KinovaArm()
        time.sleep(1)

        self.position = None  # Will be np.array([x, y, z])
        self.orientation = None  # Will be np.array([x, y, z, w])
        self.gripper =  None


    def move_to_grasp_init(self,hook_id:int):
        act_seq = get_yaml_poses(hook_num=hook_id)
        for act in act_seq:
            self.arm.move_angular(act)
            # time.sleep(1)
        self.arm.open_gripper()
        time.sleep(0.5)


    def scan_aruco(self,hook_id:int,duration=5):
        act_seq = get_yaml_poses(yaml_path="",hook_num=hook_id)
        start_time = time.time()
        aruco_pose = None
        while ((time.time() - start_time) < duration) and (aruco_pose is None):
            aruco_pose = rospy # sth
            act = act_seq[1]
            self.arm.move_angular(act)
        print("[INFO] Aruco Found")


    def grasp_strap(self, act):
        tvect = act[:2]
        rvect = act[3:]
        self.arm.move_cartesian(xyz=tvect,xyz_quat=rvect)
        time.sleep(0.5)
        self.arm.close_gripper()



    def run(self,hook_id):
        self.move_to_grasp_init(hook_id=hook_id)
        time.sleep(2)
        self.scan_aruco(hook_id=hook_id)
        time.sleep(2)
        self.grasp_strap()

        self.arm.disconnect()


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['a', 'b', 'c', 'd']:
        print("Usage: python3 script_name.py [a|b|c|d]")
        sys.exit(1)

    hook_id = sys.argv[1]

    rospy.init_node('robot_control', anonymous=True)
    task = GraspStrap()
    task.run(hook_id)