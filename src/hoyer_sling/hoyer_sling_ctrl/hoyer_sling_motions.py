#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
import sys
sys.path.append('../..')
import logging
import math


from simple_pid import PID


import time
import tf
import tf.transformations as tf_trans
import math
from simple_pid import PID
from collections import deque
import numpy as np



class Hoyer_Sling_Motions():
    def __init__(self,microcontroller_type='arduino') -> None:
        self.microcontroller_type = microcontroller_type
        self._load_modules()

        self.tf_listener = tf.TransformListener()

        # History of past transforms for interpolation
        self.pose_history = deque(maxlen=2)  # Store (timestamp, translation, rotation)

        # PID controllers for linear and angular control
        self.linear_pid = PID(Kp=25, Ki=0, Kd=0, setpoint=0)
        self.angular_pid = PID(Kp=10.0, Ki=0, Kd=0, setpoint=0)
        self.linear_pid.output_limits = (0, 15) #15
        self.angular_pid.output_limits = (-45,45)#(-45, 45)

        # Initialize current state
        self.x_current = self.y_current = self.z_current = 0.0
        self.roll_current = self.pitch_current = self.yaw_current = 0.0


    
    def _load_modules(self):
        if self.microcontroller_type == 'arduino':
            from hoyer_sling.hoyer_sling_api.arduino_interface import hoyer_sling
            self.robot = hoyer_sling()
        elif self.microcontroller_type == 'rpi':
            from hoyer_sling.hoyer_sling_api.rpi_interface import hoyer_sling
            self.robot = hoyer_sling()
        
        print(f"[Hoyer_sling/motions] Modules loaded")
        


    def _hoyer_sling_state_update(self):
        now = rospy.Time.now()
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
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


    def _move_with_pid(self, move_func, target_value, is_rotation, timeout, pid, speed_min_thresh = 3):
        start_time = time.time()
        min_loop_dt = 1.0 / 300.0  # seconds

        # Rate tracking
        last_pub_time = None
        rates = []

        # Get initial pose
        self._hoyer_sling_state_update()
        x_start = self.x_current
        y_start = self.y_current
        yaw_start = self.yaw_current

        self._first_step = True

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
            self._hoyer_sling_state_update()

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

        self.robot.stop()

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




    def run(self):
        """
        start to move the hoyer to the clicked point. hoyer's state is updated through '/tf'
        """
        self.robot.base.start()
        self.vel_cap = 70
        self.trans_err = 0.5 # meters
        self.yaw_err = 5 
        while not rospy.is_shutdown():
            # current states update
            self.hoyer_state_update()
            # Compute errors
            dx = self.x_target - self.x_current
            dy = self.y_target - self.y_current
            distance_error = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_target - self.yaw_current)
            # Compute PID outputs
            linear_velocity = -self.linear_pid(distance_error)
            angular_velocity = -self.angular_pid(angle_error)
            linear_velocity = max(min(linear_velocity, self.vel_cap), -self.vel_cap)
            angular_velocity = max(min(angular_velocity, self.vel_cap), -self.vel_cap)

            # ----- hoyer control -----
            # timeout
            if (self.flag_new_point) and (self.start_time is not None):
                if (time.time() - self.start_time) > self.time_out:
                    # self.start_time = None
                    self.robot.base.move_with(0,0)
                    # self.flag_new_point = False
                    print("\033[93m[WARN] Motion timeout\033[0m")
                    continue

                # thresholds so that the wheelchair will start re-orientation
                if self.at_target:
                    print('[INFO] Reaches destination')
                    self.start_time = None
                    self.robot.base.stop()
                    self.flag_new_point = False
                else:
                    print(f"\033[92m[INFO] Time: {time.time() - self.start_time} Moving to: ",self.x_target,",",self.y_target," from: ",self.x_current,",",self.y_current," With v: ",linear_velocity," w: ",angular_velocity,'\033[0m')
                    
                # if (self.yaw_motion_start) and ((time.time() - self.start_time) < 10):
                #     if abs(angle_error) > abs(math.radians(15)):  # 0.05 radians (~2.9 degrees) tolerance
                #         self.robot.base.move_with(-angular_velocity, angular_velocity)
                #     else:
                #         self.robot.base.move_with(0,0)
                #         print("Orientation towards target complete")
                #         self.linear_motion_start = True
                #         self.yaw_motion_start = False
                # if self.linear_motion_start:
                if True:
                    if (abs(distance_error) > 0.2) or (self.x_current > 0.1):
                        r_linear_velocity = linear_velocity
                        # if linear_velocity <= :
                        #     linear_velocity = 20
                        #     r_linear_velocity = linear_velocity+5
                        #     print("[Warn] PWM low")
                        self.robot.base.move_with(r_linear_velocity, linear_velocity)
                        print("moving")
                    else:
                        self.linear_motion_start = False
                        self.at_target = True
                        print("[INFO] Target reached!")
                time.sleep(0.25)
                
        self.robot.base.stop()

if __name__ == '__main__':
    rospy.init_node('clicked_point_sub')
    task = Hoyer_Sling_Motions()
    task.run()

