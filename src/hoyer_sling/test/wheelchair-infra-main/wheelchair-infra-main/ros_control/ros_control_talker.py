import keyboard
import socket, sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *
import rospy
from std_msgs.msg import String


# Main control loop
x = 0
y = 0



def print_helper():
    message = "Hi, welcome! To control the wheelchair, you can press:\n"
    message += " - 'w' to move forward\n"
    message += " - 's' to move backward\n"
    message += " - 'a' to turn left\n"
    message += " - 'd' to turn right"

    print(message)


## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic 


def talker():
    pub = rospy.Publisher('movement', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        if keyboard.is_pressed("w"):
            y = 100
        elif keyboard.is_pressed("s"):
            y = 180
        else:
            y = 0

        if keyboard.is_pressed("a"):
            x = 180
        elif keyboard.is_pressed("d"):
            x = 100
        else:
            x = 0
        # print(x, y)

        hello_str = str(x) + "#" + str(y) + "# time: %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        print_helper()
        talker()
    except rospy.ROSInterruptException:
        pass