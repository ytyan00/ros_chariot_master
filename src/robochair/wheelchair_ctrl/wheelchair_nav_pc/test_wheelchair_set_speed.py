#!/usr/bin/env python3

import sys
sys.path.append('../..')
import time
import random

from wheelchair_pc import Wheelchair
from collections import deque
import numpy as np

class MotionCommander:
    def __init__(self):
        self.robot = Wheelchair()

        # History of past transforms for interpolation
        self.pose_history = deque(maxlen=2)  # Store (timestamp, translation, rotation)


        # Initialize current state
        self.x_current = self.y_current = self.z_current = 0.0
        self.roll_current = self.pitch_current = self.yaw_current = 0.0



    def run(self):
        """
        move_func: function to send PWM command (e.g., self.robot.v_forward or self.robot.rotate)
        target_value: meters (translation) or radians (rotation)
        is_rotation: True if rotation, False if translation
        pid: PID controller (e.g., self.angular_pid or self.linear_pid)
        """
        start_time = time.time()
        min_loop_dt = 1.0 / 300.0  # seconds

        while  True:
            loop_start = time.time()

            # self.robot.v_forward(min(max(0,random.random()*20),22))
            self.robot.v_forward(1)
            loop_duration = time.time() - loop_start
            if loop_duration < min_loop_dt:
                time.sleep(min_loop_dt - loop_duration)


    
    def warm_up(self):
        time.sleep(2)

if __name__ == "__main__":
    commander = MotionCommander()
    commander.warm_up()
    commander.run()