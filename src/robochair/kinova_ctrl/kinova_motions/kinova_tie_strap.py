import sys
import os
import numpy as np

import rospy
import rospkg
import time
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse


rospack = rospkg.RosPack()
pkg_path = rospack.get_path('ros_chariot_master')
kinova_path = os.path.join(pkg_path, 'src/robochair/kinova_ctrl')
sys.path.append(kinova_path)
from kinova import KinovaArm
from utils.kinova_get_yaml_pose import get_yaml_poses


rospack = rospkg.RosPack()
pkg_path = rospack.get_path('ros_chariot_master')
aruco_path = os.path.join(pkg_path, 'src/robochair/chariot_aruco_marker')
sys.path.append(aruco_path)
from aruco_kinova_trans import ArucoBaseTransformer

class Tie_Strap:
    def __init__(self,robot:KinovaArm):
        """
        run on robochair, listen to policy_server mpc output and control the konova arm
        """
        self.robot = robot 
        self.aruco_pose = ArucoBaseTransformer()

        self.target = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        

    def parse_pose(self, pose_stamped):
        """
        Extract position and orientation (Euler) from a PoseStamped.
        Returns: (x, y, z, theta_x, theta_y, theta_z) in radians
        """
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        theta_x, theta_y, theta_z = euler_from_quaternion(quaternion)

        return pos.x, pos.y, pos.z, theta_x, theta_y, theta_z


    def lookup_transform(self, base_frame="base_link", child_frame="tool_frame", timeout=1.0):
        """
        Looks up the latest transform from `child_frame` to `base_frame`.

        Args:
            base_frame (str): The reference frame (e.g., 'base_link').
            child_frame (str): The target frame (e.g., 'tool_frame').
            timeout (float): Timeout in seconds for the TF lookup.

        Returns:
            TransformStamped if successful, else None.
        """


        try:
            rospy.sleep(1.0)  # Give some time to populate the TF tree
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                child_frame,
                rospy.Time(0),  # latest available
                rospy.Duration(timeout)
            )
            return transform
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return None



    def get_increment_to_target(self, target_pose_stamped, tool_frame="tool_frame", base_frame="base_link"):
        """
        Compute the incremental motion needed to move from the current tool_frame pose
        to the target_pose_stamped, both in base_link frame.

        Returns:
            [dx, dy, dz, dtheta_x, dtheta_y, dtheta_z] in meters and radians
        """
        # 1. Lookup the current pose of the tool_frame w.r.t. base_link
        tool_pose = self.lookup_transform(base_frame=base_frame,child_frame=tool_frame)

        # 2. Extract current position and orientation
        trans = tool_pose.transform.translation
        rot = tool_pose.transform.rotation
        current_position = [trans.x, trans.y, trans.z]
        current_quat = [rot.x, rot.y, rot.z, rot.w]


        # 3. Extract target position and orientation (in base frame)
        tp = target_pose_stamped.pose.position
        to = target_pose_stamped.pose.orientation
        target_position = [tp.x, tp.y, tp.z]
        target_quat = [to.x, to.y, to.z, to.w]

        # 4. Compute position difference
        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        dz = target_position[2] - current_position[2]

        # 5. Compute relative rotation: q_relative = q_target * q_current⁻¹
        q_current_inv = quaternion_inverse(current_quat)
        q_relative = quaternion_multiply(target_quat, q_current_inv)

        # 6. Convert relative rotation to Euler angles
        dtheta_x, dtheta_y, dtheta_z = euler_from_quaternion(q_relative)

        return [dx, dy, dz, 0, 0, 0]

    def move_to_grasp_init(self,hook_id:int,yaml_path='/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl/config/grasp_init_pos.yaml'):
        act_seq = get_yaml_poses(yaml_path=yaml_path,hook_num=hook_id)
        for act in act_seq:
            self.robot.move_angular(act)
            # time.sleep(1)
        self.robot.open_gripper()
        time.sleep(0.5)

    def scan_aruco(self, hook_id: int, yaml_path="/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl/config/scan.yaml"):
        axis_ranges = get_yaml_poses(yaml_path=yaml_path, hook_num=hook_id)

        # Identify scan axis and parameters from YAML: use [offset_min, offset_max, duration]
        axis = None
        for i, r in enumerate(axis_ranges):
            if isinstance(r, list) and len(r) == 3:
                axis = i
                offset_min, offset_max, duration = r
                break

        if axis is None:
            raise ValueError(f"No valid scan axis and duration found for hook {hook_id}")

        # Get current pose
        _, ee_pose, _ = self.robot.get_state()
        base_tvect = ee_pose[:3]
        rvect = ee_pose[3:]
        current_value = base_tvect[axis]

        # Compute actual scan limits based on current pose
        min_limit = current_value + offset_min
        max_limit = current_value + offset_max

        print(f"[INFO] Scanning axis {axis} from {min_limit:.3f} to {max_limit:.3f} (offsets {offset_min}, {offset_max}) for {duration}s")

        # Define scan steps
        step_size = 0.05  # 5 cm resolution
        scan_positions = np.arange(min_limit, max_limit + step_size, step_size)

        start_time = time.time()
        aruco_pose = None

        for pos in scan_positions:
            # Set target vector and move
            tvect = base_tvect.copy()
            tvect[axis] = pos
            self.robot.move_cartesian(xyz=tvect, xyz_quat=rvect)
            time.sleep(0.2)  # wait for motion + sensor update

            # Check for ArUco marker
            aruco_pose = self.aruco_pose.pose
            if aruco_pose is not None:
                tp = aruco_pose.pose.position
                target_position = [tp.x, tp.y, tp.z]
                to = aruco_pose.pose.orientation
                target_quat = [to.x, to.y, to.z, to.w]
                print("[INFO] Aruco Pose Found at", target_position)
                return target_position, target_quat

            if (time.time() - start_time) > duration:
                break

        raise RuntimeError("Aruco pose not found within scan range and duration.")

    def grasp_strap(self,target_position,target_quat=None):
        """
        Leave target_quat as None if assume the robot's orientation 
        is ready to grasp and to ignore marker orientation
        """
        if target_quat is None:
            # assume the robot's orientation is ready to grasp
            # ignore marker orientation
            _,ee_pose,_ = self.robot.get_state() 
            target_quat = ee_pose[3:]

        
        # target_pos x offset, ee should grasp 3cm outward from the marker
        # target_position z is set to the lower limite of the robot
        target_position[0] = target_position[0] - 0.032
        target_position[2] = -0.25112566
        self.robot.move_cartesian(target_position,target_quat)
        time.sleep(1)
        self.robot.close_gripper()

    def grasp_and_retract(self, hook_id, yaml_path='/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl/config/retract.yaml'):
        _,ee_pos,_ = self.robot.get_state()
        tvect = ee_pos[:3]
        rvect = ee_pos[3:]
                
        retraction_offset = get_yaml_poses(yaml_path=yaml_path,hook_num=hook_id)
        for i,offset in enumerate(retraction_offset):
            tvect[i] = tvect[i] + offset
        self.robot.move_cartesian(xyz=tvect,xyz_quat=rvect)

        final_pos_seq = get_yaml_poses(yaml_path=yaml_path,hook_num=(0-hook_id))
        for act in final_pos_seq:
            self.robot.move_angular[act]




    def run(self,hook_id, cmd='full'):

        if cmd == 'full':
            # step 1, ee approach the bed
            self.move_to_grasp_init(hook_id=hook_id)

            # step 2, look for aruco
            target_p,_ = self.scan_aruco(hook_id=hook_id)

            # step 3, approach strap and grasp
            self.grasp_strap(target_position=target_p)

            # # step 4, hold strap and retract

            # self.grasp_and_retract(hook_id)

            print("[robochair/Kinova] Grasping finished")
            return cmd, True
        
        elif cmd == '(INOP.)':
            return cmd + ':' +'1'

    

            

if __name__ == "__main__":
    rospy.init_node('kinova_grasp_strap')
    ex = Tie_Strap(KinovaArm())
    ex.run(1)