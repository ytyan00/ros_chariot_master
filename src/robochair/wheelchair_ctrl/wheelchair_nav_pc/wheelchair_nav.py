#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
import sys
sys.path.append('../..')
sys.path.append("/home/chariot/catkin_ws/src/ros_chariot_master/src/robochair/wheelchair_ctrl/wheelchair_api/")

import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import time
import tf
import tf.transformations as tf_trans
import math
from simple_pid import PID
from wheelchair_api.wheelchair_pc import Wheelchair
from collections import deque
import numpy as np
from wheelchair_ctrl.utils.wheelchair_load_pid import wheelchair_load_pid_config

class WC_MotionCommander:
    def __init__(self):
        self.robot = Wheelchair()
        self.tf_listener = tf.TransformListener()

        # History of past transforms for interpolation
        self.pose_history = deque(maxlen=2)  # Store (timestamp, translation, rotation)

        # Load PID
        self._wheelchair_set_pid()

        # Initialize current state
        self.x_current = self.y_current = self.z_current = 0.0
        self.roll_current = self.pitch_current = self.yaw_current = 0.0

    def _wheelchair_set_pid(self):
        # PID controllers
        pid_config = wheelchair_load_pid_config()

        fwd = pid_config['forward']
        self.forward_pid = PID(Kp=fwd['p'], Ki=fwd['i'], Kd=fwd['d'], setpoint=0)
        self.forward_pid.output_limits = (35, 100)

        bwd = pid_config['backward']
        self.backward_pid = PID(Kp=bwd['p'], Ki=bwd['i'], Kd=bwd['d'], setpoint=0)
        self.backward_pid.output_limits = (35, 100)

        ang = pid_config['angular']
        self.angular_pid = PID(Kp=ang['p'], Ki=ang['i'], Kd=ang['d'], setpoint=0)
        self.angular_pid.output_limits = (-100, 100)

    def _wheelchair_state_update(self):
        now = rospy.Time.now()
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/wheelchair_base_link', rospy.Time(0))
            self.pose_history.append((now.to_sec(), trans, rot))

            self.x_current, self.y_current, self.z_current = trans
            self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if len(self.pose_history) < 2:
                rospy.logwarn("TF unavailable and not enough data for interpolation.")
                return  # Not enough data to interpolate

            # Get the last two poses
            (t1, trans1, rot1), (t2, trans2, rot2) = self.pose_history

            if t2 == t1:
                rospy.logwarn("Timestamps are equal, skipping interpolation.")
                return

            t_now = now.to_sec()
            ratio = (t_now - t1) / (t2 - t1)

            interp_trans = [
                trans1[i] + ratio * (trans2[i] - trans1[i]) for i in range(3)
            ]

            # Interpolate orientation (slerp for rotation)
            q1 = np.array(rot1)
            q2 = np.array(rot2)
            interp_rot = tf_trans.quaternion_slerp(q1, q2, ratio)

            # Update current pose
            self.x_current, self.y_current, self.z_current = interp_trans
            self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(interp_rot)

    def _move_with_pid(self, move_func, target_value, is_rotation, timeout, pid, speed_min_thresh = 2):
        start_time = time.time()
        min_loop_dt = 1.0 / 300.0  # seconds

        # Rate tracking
        last_pub_time = None
        rates = []

        # Get initial pose
        self._wheelchair_state_update()
        x_start = self.x_current
        y_start = self.y_current
        yaw_start = self.yaw_current

        self._first_step = True

        try:
            while not rospy.is_shutdown():
                STATIC_PWM = 25
                KICK_DURATION = 0.05

                loop_start = time.time()
                elapsed = loop_start - start_time

                # Timeout check
                if elapsed > timeout:
                    rospy.logwarn("[WARN] Timeout reached.")
                    break

                # Update current pose
                self._wheelchair_state_update()

                # Compute actual progress
                if is_rotation:
                    yaw_progress = math.degrees(self._normalize_angle(self.yaw_current - yaw_start))
                    error = yaw_progress - target_value
                    if abs(error) < 2:
                        rospy.loginfo("[INFO] Reached target.")
                        break
                else:
                    dx = self.x_current - x_start
                    dy = self.y_current - y_start
                    distance_progress = (dx ** 2 + dy ** 2) ** 0.5
                    error = distance_progress - target_value

                    if abs(error) < 0.025:
                        rospy.loginfo("[INFO] Reached target.")
                        break

                pwm = pid(error)
                pwm = abs(pwm)
                if abs(pwm) < speed_min_thresh:
                    speed_min_thresh

                if self._first_step and pwm > 0:
                    pwm = max(pwm, STATIC_PWM)
                    if elapsed >= KICK_DURATION:
                        self._first_step = False

                move_func(abs(pwm))  # <- actual control command

                # --- Calculate and print rate ---
                current_time = time.time()
                if last_pub_time is not None:
                    dt = current_time - last_pub_time
                    rate = 1.0 / dt
                    rates.append(rate)
                    print(f'Control rate: {rate:.2f} Hz')
                last_pub_time = current_time
                # --------------------------------

                print(f'error: {error:.4f}, pwm: {pwm:.2f}')

                # Limit loop to 300 Hz
                loop_duration = time.time() - loop_start
                if loop_duration < min_loop_dt:
                    time.sleep(min_loop_dt - loop_duration)

        finally:
            self.robot.stop()
            rospy.loginfo("[INFO] Automatic stop issued.")

        # Optional: average control rate
        if rates:
            avg_rate = sum(rates) / len(rates)
            print(f'[INFO] Average control rate: {avg_rate:.2f} Hz')

        # Pause after motion
        sleep_duration = 20
        sleep_start = time.time()
        while not rospy.is_shutdown() and (time.time() - sleep_start < sleep_duration):
            time.sleep(0.1)


    def _normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi]
        """
        import math
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def execute_command(self, action, value, timeout=30):
        action = action.lower()
        self._first_step = True
        if action == "forward":
            rospy.loginfo(f"[INFO] Forward {value}m (timeout: {timeout}s)")
            self._move_with_pid(self.robot.v_forward, value, False, timeout, self.forward_pid)

        elif action == "backward":
            rospy.loginfo(f"[INFO] Backward {value}m (timeout: {timeout}s)")
            self._move_with_pid(self.robot.v_backward, value, False, timeout, self.backward_pid)

        elif action == "left":
            rospy.loginfo(f"[INFO] Turn Left {value}° (timeout: {timeout}s)")
            self._move_with_pid(self.robot.v_left, value, True, timeout, self.angular_pid)

        elif action == "right":
            rospy.loginfo(f"[INFO] Turn Right {value}° (timeout: {timeout}s)")
            # self.angular_pid.setpoint = 0
            self._move_with_pid(self.robot.v_right, value, True, timeout, self.angular_pid)

        elif action == "stop":
            self.robot.stop()
            rospy.loginfo("[INFO] Stopped.")

        elif action == "hold":
            self.robot.press_key_to_continue()

        else:
            rospy.logwarn(f"[WARN] Unknown action: {action}")

    def execute_sequence(self, sequence):
        
        i = 0
        while i < len(sequence):
            action = sequence[i][0]
            value = sequence[i][1]
            timeout = 30  # default timeout

            if i + 1 < len(sequence) and isinstance(sequence[i + 1], tuple) and sequence[i + 1][0] == "timeout":
                timeout = sequence[i + 1][1]
                i += 1

            self.execute_command(action, value, timeout)
            i += 1

        print("[INFO] Sequence complete.")
        self.robot.stop()
    
    def warm_up(self):
        time.sleep(2)

    def run(self, hook_id:int = 0,cmd:str = ""):
        if hook_id == 1:
            sequence = [
                ("backward", 0.6), ("timeout", 10),
                ("left", 80),   ("timeout", 10),
                ("forward", 0.85), ("timeout", 10),
                ("right", -86),   ("timeout", 10),
                ("forward", 2.9), ("timeout", 10),
                ("right", -84),   ("timeout", 10),
                ("forward", 0.35), ("timeout", 10),
                ("forward", 1), ("timeout", 10),
                ("forward", 1.5), ("timeout", 30),
            ]
        elif hook_id == 2:
            sequence = [
                ("backward", 0.6), ("timeout", 10),
                ("left", 80),   ("timeout", 10),
                ("forward", 0.85), ("timeout", 10),
                ("right", -86),   ("timeout", 10),
                ("forward", 2.9), ("timeout", 10),
                ("right", -84),   ("timeout", 10),
                ("forward", 0.35), ("timeout", 10),
                ("forward", 1), ("timeout", 10),
                ("forward", 1.5), ("timeout", 30),
            ]        

        commander.execute_sequence(sequence)

        return cmd, True

if __name__ == "__main__":
    rospy.init_node('wheelchair_motion_commander')
    commander = WC_MotionCommander()
    commander.warm_up()
    commander.run(1)
