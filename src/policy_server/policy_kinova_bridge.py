import sys
import os
import time

import rospy

class KinovaBridge:
    def __init__(self):
        """
        Communicate between cart-mpc policy and robot control command
        
        """
        pass

    def run(self,hook_id:int,cmd='full'):
        return "INOP.", False