#!/usr/bin/env python3
import rospy
from wheelchair_nav_v1 import MotionCommander  # Make sure this is the correct import path
import time

def test_state_update():
    commander = MotionCommander()

    while not rospy.is_shutdown():
        commander._wheelchair_state_update()
        print(f"Time: {time.time()}; Position: x={commander.x_current:.2f}, y={commander.y_current:.2f}, yaw={commander.yaw_current:.2f}")

if __name__ == "__main__":
    try:
        rospy.init_node('test_wheelchair_state_update')
        test_state_update()
    except rospy.ROSInterruptException:
        pass
