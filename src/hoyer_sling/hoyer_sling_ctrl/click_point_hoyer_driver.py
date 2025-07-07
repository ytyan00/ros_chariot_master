#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
import sys
sys.path.append('../..')
import logging
import time
import tf
import tf.transformations as tf_trans
import math

import hoyer_sling
from simple_pid import PID


class camera_hoyer_teleop():
    def __init__(self) -> None:
        self.robot = hoyer_sling.hoyer_sling()
    
        # camera point subscriber
        self.flag_new_point = False
        self.y_current = self.x_current = self.z_current = self.yaw_current = self.x_target = self.y_target = self.z_target = 0    
        self.start_time = None
        self.at_target = False
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        # timeout in seconds for moving the hoyer base
        self.time_out = 17
        # PID controllers for linear and angular control
        self.linear_pid = PID(Kp=25, Ki=0, Kd=0, setpoint=0)
        self.angular_pid = PID(Kp=10.0, Ki=0, Kd=0, setpoint=0)
        self.linear_motion_start = False
        self.yaw_motion_start = True
        self.re_orientation_yaw = 0


    def point_callback(self,msg:PointStamped):
        try:
            # x(Fwd,Bwd), y(R,L), z(Up, Down)
            self.x_target, self.y_target, self.z_target = msg.point.x, msg.point.y, msg.point.z
            self.x_target += 0.6
            print(f"Clicked point: {msg.point.x}, {msg.point.y}, {msg.point.z}")
            rospy.loginfo("Received target forward: {}, left: {}".format(self.x_target, self.y_target))
            self.flag_new_point = True
            self.start_time = time.time()
        except Exception as e:
            rospy.logerr("Error in wheelchair_callback: {}".format(e))

    def hoyer_state_update(self):
        """ 
        hoyer's states update through tf
        """
        try:
            (current_trans, current_rot) = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
        except:
            return
        # we assume that camera position start at world origin
        self.x_current, self.y_current, self.z_current = current_trans
        self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(current_rot) # Convert quaternion to Euler angles
        # rospy.loginfo("Current position forward: {}, left: {}".format(self.x_current, self.y_current))
    
    # helper functions
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def rotate_to(self,yaw_target:float,timeout=5)->None:
        start_time = time.time()
        if not (time.time() - start_time > timeout):
            yaw_err = self.normalize_angle(yaw_target-self.yaw_current)
            angular_velocity = -self.angular_pid(yaw_err)
        else:
            self.robot.base.move_with(0,0)
            print("[WARN] rotate_to time out")


    

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
    task = camera_hoyer_teleop()
    task.run()

