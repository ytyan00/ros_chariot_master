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

    
        # camera point subscriber
        self.flag_new_point = False
        self.y_current = self.x_current = self.z_current = self.yaw_current = self.x_target = self.y_target = self.z_target = 0    
        self.start_time = None
        self.at_target = False
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        # timeout in seconds for moving the hoyer base
        self.time_out = 17
        self.attemp = 0



    def point_callback(self,msg:PointStamped):
        try:
            # x(Fwd,Bwd), y(R,L), z(Up, Down)
            self.x_target, self.y_target, self.z_target = msg.point.x, msg.point.y, msg.point.z
            self.x_target += 0.6
            print(f"Clicked point: {msg.point.x}, {msg.point.y}, {msg.point.z}")
            # rospy.loginfo("Received target forward: {}, left: {}".format(self.x_target, self.y_target))
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
            self.attemp += 1
        except:
            return
        # we assume that camera position start at world origin
        self.x_current, self.y_current, self.z_current = current_trans
        self.roll_current, self.pitch_current, self.yaw_current = tf_trans.euler_from_quaternion(current_rot) # Convert quaternion to Euler angles
        # rospy.loginfo("Current position forward: {}, left: {}".format(self.x_current, self.y_current))
        print(f"x_current: {self.x_current}, y_current: {self.y_current}, z_current: {self.z_current}")
    
    # helper functions
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    



    

    def run(self):
        """
        start to move the hoyer to the clicked point. hoyer's state is updated through '/tf'
        """
        self.vel_cap = 70
        self.trans_err = 0.5 # meters

        start = time.time()
        while ((time.time()-start)<=1):
            # current states update
            self.hoyer_state_update()
        print(self.attemp)
        

  

if __name__ == '__main__':
    rospy.init_node('clicked_point_sub')
    task = camera_hoyer_teleop()
    task.run()

