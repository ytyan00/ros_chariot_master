import rospy
import time
import sys
from std_msgs.msg import String
from kinova_control.kinova_grasp_strap import GraspStrap






class Task_Plan:
    def __init__(self):
        self.wc_pub = rospy.Publisher('/wheelchair_cmd', String, queue_size=10)
        self.kinova_pub = rospy.Publisher('/kinova_cmd', String, queue_size=10)
        self.pl_pub = rospy.Publisher('/wheelchair_cmd', String, queue_size=10)
    def __localization(self):
        pass

    def start_strap_grasp(self):
        self.wc_pub.publish("strap_grasp")
