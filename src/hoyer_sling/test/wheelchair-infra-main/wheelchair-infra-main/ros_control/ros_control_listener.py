## Simple talker demo that listens to std_msgs/Strings published 
## to the 'movement' topic

import rospy
from std_msgs.msg import String
import sys
sys.path.append('..')
from python_interface import wheelchair
import logging
import threading
import time

def callback(data):
    global wheelchair
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    raw_value = data.data
    data = raw_value.split("#")
    x = data[0]
    y = data[1]
    wheelchair.set_direction(x, y)

def always_move(wheelchair):
    while True:
        wheelchair.move()




def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('movement', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    wheelchair = wheelchair.Wheelchair()
    wheelchair.start_steaming()

    x = threading.Thread(target=always_move, args=(wheelchair,))
    x.start()

    
    listener()