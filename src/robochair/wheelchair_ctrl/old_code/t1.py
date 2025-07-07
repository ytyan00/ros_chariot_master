#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
import sys
sys.path.append('../..')
import logging
import time
import tf
import tf.transformations as tf_trans
import math
import pc_wheelchair_control_pub
from simple_pid import PID

class camera_wheelchair_teleop():
    def __init__(self) -> None:
        # Current position and target position
        self.x_current = self.y_current = self.z_current = 0
        self.yaw_current = 0
        self.x_target = self.y_target = 0
        self.yaw_target = 0
        
        self.tf_listener = tf.TransformListener()
        # Create a publisher for the /wheelchair_target topic
        self.pub = rospy.Publisher('/wheelchair_control', String, queue_size=10)
        print("[INFO] ROS topic init")
        time.sleep(5)
        print("[INFO] All is well")
        # PID controllers
        self.linear_pid = PID(Kp=15, Ki=0, Kd=1, setpoint=0)
        self.angular_pid = PID(Kp=90.0, Ki=0, Kd=2, setpoint=0)

        # Movement flags and timers
        self.movement_complete = False
        self.stage = 0
        self.stage_last = -1
        self.movement_start_time = None

        self.attampt = 0

    def hoyer_state_update(self):
        """ 
        Update current state through tf
        """
        try:
            (current_trans, current_rot) = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
        except:
            return
        self.x_current, self.y_current, self.z_current = current_trans
        self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(current_rot)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def execute_trajectory(self):
        """
        Define the trajectory and execute step by step with timeouts.
        """
        # Define the sequence of movements for the wheelchair, each with a specific timeout
        # trajectory_steps = [
        #     {"type": "rotate", "angle": math.radians(30), "timeout": 20},  # Turn left 30 degrees, 3-second timeout
        #     {"type": "move", "distance": 0.75, "timeout": 2},             # Move forward 0.75 meters, 2-second timeout
        #     {"type": "rotate", "angle": math.radians(-90), "timeout": 50}  # Turn right 90 degrees, 2-second timeout
        # ]
        trajectory_steps = [
            # new trajectory
            {"type": "move_x", "distance": -0.61, "timeout": 50},  # Turn left 30 degrees, 3-second timeout
            {"type": "rotate", "angle": math.radians(-90), "timeout": 50},
            {"type": "move_y", "distance": 1, "timeout": 50},  # Turn left 30 degrees, 3-second timeout
            {"type": "rotate", "angle": math.radians(90), "timeout": 50},
            {"type": "move_x", "distance": 2.9, "timeout": 50}, 
            {"type": "rotate", "angle": math.radians(85), "timeout": 50},
            {"type": "move_y", "distance": -1.67, "timeout": 50}, 
            {"type": "move_y", "distance": -0.63, "timeout": 50}, 

        ]
#            {"type": "move_y", "distance": -1.1, "timeout": -13},  # -> move to location 4
#            {"type": "rotate", "angle": math.radians(-82.5), "timeout": 50},  # Turn left 30 degrees, 3-second timeout
        if self.stage < len(trajectory_steps):
            current_step = trajectory_steps[self.stage]
            timeout_duration = abs(current_step.get("timeout", 3))  # Default to 3 seconds if not specified
            self.vel_direction = current_step.get("timeout", 3)/timeout_duration
            # Initialize timer for the step if not started
            if self.movement_start_time is None:
                self.movement_start_time = time.time()

            # Check for timeout
            if time.time() - self.movement_start_time > timeout_duration:
                print("\033[93m[WARN] Movement timeout, moving to next step\033[0m")
                self.movement_complete = True
            if not self.movement_complete:
                if current_step["type"] == "rotate":
                    if self.stage > self.stage_last:
                        # self.rotate_by = current_step["angle"] + self.yaw_current
                        self.rotate_by = self.yaw_current - current_step["angle"] 
                        self.stage_last = self.stage
                    else:
                        self.rotate_ttarget_valueo_angle(self.rotate_by)
                elif current_step["type"] == "move_x":
                    if self.stage > self.stage_last:
                        self.trans_by = current_step["distance"] + self.x_current
                        self.stage_last = self.stage
                    else:
                        self.move_forward(self.trans_by,"x")
                elif current_step["type"] == "move_y":
                    if self.stage > self.stage_last:
                        self.trans_by = current_step["distance"] + self.y_current
                        self.stage_last = self.stage
                    else:
                        self.move_forward(self.trans_by,"y")

            if self.movement_complete:
                # Sleep for 5 seconds after each step is completed
                time.sleep(20)
                self.stage += 1
                self.movement_complete = False
                self.movement_start_time = None  # Reset timer for the next step

    def rotate_to_angle(self, target_angle):
        """
        Rotate the wheelchair to the specified target angle with a timeout.
        """
        # Update current states
        self.hoyer_state_update()
        yaw_cap = 80
        yaw_cap_low = 30
        # Compute angle error
        angle_error = self.normalize_angle(target_angle - self.yaw_current)
        angular_velocity = - self.angular_pid(angle_error)
        angular_velocity = max(-yaw_cap, min(yaw_cap, angular_velocity))
        if abs(angular_velocity) < yaw_cap_low and abs(angular_velocity) > yaw_cap_low - 5:
            angular_velocity = yaw_cap_low if angular_velocity > 0 else -yaw_cap_low


        # Apply rotation
        if abs(angle_error) > math.radians(5):  # 5-degree tolerance
            command = f"0,{angular_velocity}"
            msg = String(data=command)
            print("Delta_theta: ", angle_error)
            rospy.loginfo(f"Publishing movement command: {command}")
            self.pub.publish(msg)
            self.attampt += 1
            time.sleep(0.085)
        else:
            command = f"0,0"
            msg = String(data=command)
            rospy.loginfo(f"Publishing movement command: {command}")
            self.pub.publish(msg)
            self.attampt += 1
            print("\033[92m[INFO] Rotation complete, error is: \033[0m",angle_error)
            self.movement_complete = True
            time.sleep(0.085)

    def move_forward(self, distance,axis):
        """
        Move the wheelchair forward by a specified distance with a timeout.
        """
        # Update current states
        self.hoyer_state_update()
        fwd_cap = 10
        # fwd_cap_low = 3
        # Compute distance error
        if axis == "x":
            current_pos = self.x_current
        elif axis == "y":
            current_pos = self.y_current
        dx = distance - current_pos
        distance_error = dx

        # Apply linear velocity
        if abs(distance_error) > 0.05:  # Tolerance for position
            linear_velocity = (- self.linear_pid(distance_error))*self.vel_direction
            linear_velocity = max(-fwd_cap, min(fwd_cap, linear_velocity))
            # if abs(linear_velocity) < fwd_cap_low and abs(linear_velocity) > fwd_cap_low - 1:
            #     linear_velocity = fwd_cap_low if linear_velocity > 0 else -fwd_cap_low
            if axis == "y":
                linear_velocity = abs(linear_velocity)

            command = f"{linear_velocity},0"
            msg = String(data=command)
            print("Delta_x: ", distance_error)
            rospy.loginfo(f"Publishing movement command: {command}")
            self.pub.publish(msg)
            self.attampt += 1
            time.sleep(0.085)
        else:
            command = f"0,0"
            msg = String(data=command)
            rospy.loginfo(f"Publishing movement command: {command}")
            self.pub.publish(msg)
            self.attampt += 1
            print("\033[92m[INFO] Forward movement complete, error is: \033[0m",distance_error)
            self.movement_complete = True
            time.sleep(0.085)


    def run(self):
        """
        Main control loop to execute the trajectory.
        """
        self.vel_cap = 70

        while not rospy.is_shutdown():
            self.execute_trajectory()
            
            # time.sleep(0.1)  # Adjust sleep time if necessary


if __name__ == '__main__':
    rospy.init_node('wheelchair_teleop')
    task = camera_wheelchair_teleop()
    task.run()
