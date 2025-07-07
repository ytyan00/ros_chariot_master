## Simple talker demo that listens to std_msgs/Strings published 
## to the 'movement' topic

import rospy
from geometry_msgs.msg import Point
import sys
sys.path.append('../..')
from python_interface import wheelchair
import logging
import threading
import time
import tf
import tf.transformations as tf_trans

import math

tf_listener = None

def wheelchair_callback(data):
    global wheelchair, tf_listener
    try:
      
        # Extract x, y, z from Point message: world coordinate
        x, y, z = data.x, data.y, data.z
        # z forward
        # x leftward
        # y up and down, dont know and dont care
        rospy.loginfo("Received target forward: {}, left: {}".format(z, x))
        if x == 0 and y == 0 and z == 0:
            return
        
        (trans, rot) = tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
        # because we assume that camera position start at world origin
        x_current, y_current, z_current = trans
        print("current position forward: {}, left: {}".format(x_current, y_current))

        # Convert quaternion to Euler angles
        euler = tf_trans.euler_from_quaternion(rot)
        roll, pitch, yaw = euler


        # # x forward positive
        # # y leftward positive
        # # z upward positive
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
        number = x - y_current
        arctan_result = math.atan(number)
        rospy.loginfo("Current orientation arctan: {} yaw: {}".format(arctan_result, yaw))
        
        if arctan_result > yaw:
            direction_y = 170
        else:
            direction_y = 70
        

        # # a threshold so that the wheelchair will stop
        if abs(z - x_current) < 1 and abs(x - y_current) < 0.5:
            print('reach destination')
            return

        wheelchair.set_direction(direction_x, direction_y)

        start_time = time.time()
        rospy.loginfo("wheelchair move foward: {}, rightward: {}".format(direction_x, direction_y))
        
        while (time.time() - start_time) < 0.25:  # Run for delta_time
            wheelchair.move()


    except Exception as e:
        rospy.logerr("Error in wheelchair_callback: {}".format(e))



def listener():
    global tf_listener
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    tf_listener = tf.TransformListener()
    rospy.Subscriber('target', Point, wheelchair_callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    wheelchair = wheelchair.Wheelchair()
    wheelchair.start_steaming()
    # wheelchair.set_direction(0,100)
    
    # while True:
    #     wheelchair.move()
    
    listener()
    
    
