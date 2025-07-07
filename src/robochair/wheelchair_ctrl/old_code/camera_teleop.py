#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import sys
sys.path.append('../..')
import logging
import time
import tf
import tf.transformations as tf_trans

import math

import hoyer_sling




class camera_hoyer_teleop():
    def __init__(self) -> None:
        rospy.init_node('camera_robot_control', anonymous=True)
        self.robot = hoyer_sling.hoyer_sling()
    
        # camera pin pont subscriber
        self.flag_new_point = False
        self.y_current = self.x_current = self.z_current = self.yaw_current = self.x_target = self.y_target = self.z_target = 0    
        self.start_time = None
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/target', Point, self.target_callback)


    def target_callback(self,data):
        try:
            # Extract x, y, z from Point message: world coordinate
            self.x_target, self.y_target, self.z_target = data.x, data.y, data.z
            # z forward, x leftward, y up and down (dont know and dont care).
            rospy.loginfo("Received target forward: {}, left: {}".format(self.z_target, self.x_target))
            self.flag_new_point = True
            self.start_time = time.time()
            
            (current_trans, current_rot) = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
            # because we assume that camera position start at world origin
            self.x_current, self.y_current, self.z_current = current_trans
            self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(current_rot) # Convert quaternion to Euler angles
            rospy.loginfo("current position forward: {}, left: {}".format(self.x_current, self.y_current))

            # # x forward positive, y leftward positive, z upward positive
            direction_x = 70
            direction_y = 0
            # direction_x  foward
            # direction_ y rightward

            # if z > x_current:
            #     direction_x = 100
            # else:
            #     direction_x = 157

            # if x > y_current:
            #     direction_y = 157
            # else:
            #     direction_y = 100


            # Example usage of atan
            arctan_result = math.atan(self.x_target - self.y_current)
            rospy.loginfo("Current orientation arctan: {} yaw: {}".format(arctan_result, self.yaw_current))
            
            if arctan_result > self.yaw_current:
                direction_y = 170
            else:
                direction_y = 70
            


            rospy.loginfo("wheelchair move foward: {}, rightward: {}".format(direction_x, direction_y))
            


        except Exception as e:
            rospy.logerr("Error in wheelchair_callback: {}".format(e))

    def run(self):
        """
        start to move the hoyer by pinning camera images
        """
        self.robot.base.start()
        while not rospy.is_shutdown():

            time_out = 5 # timeout in seconds for moving
            if (time.time() - self.start_time) < time_out:
                # thresholds so that the wheelchair will stop
                if (abs(self.z_target - self.x_current) < 1) and (abs(self.x_target - self.y_current) < 0.5):
                    print('[INFO] Reaches destination')
                    self.start_time = None
                    self.robot.base.stop()
                else:
                    print("moving to: ",self.z_target,",",self.x_target,"from: ",self.x_current,self.y_current)
            else:
                print("[WARN] motion timeout")
        self.robot.base.stop()






if __name__ == '__main__':
    task = camera_hoyer_teleop()
    task.run()
    
    
