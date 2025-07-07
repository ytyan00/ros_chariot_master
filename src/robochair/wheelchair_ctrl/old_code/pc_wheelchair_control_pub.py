#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time
import sys
import select
import termios
import tty

class WheelchairControlPublisher:
    """
    Keyboard-based control to publish movement commands ('w', 'a', 's', 'd') to the /wheelchair_target topic.
    """

    def __init__(self):
        # Create a publisher for the /wheelchair_target topic
        self.pub = rospy.Publisher('/wheelchair_control', String, queue_size=10)
        time.sleep(1)

    def wheelchair_publish(self, command: str):
        """
        Publish the movement command (key press) to the /wheelchair_target topic.
        """
        msg = String(data=command)
        rospy.loginfo(f"Publishing movement command: {command}")
        self.pub.publish(msg)

    def run(self):
        """
        Main loop to capture keyboard input and publish corresponding commands.
        """
        print("NA")
        # try:
        #     while not rospy.is_shutdown():
        #         key = self.get_key()
        #         if key in ['w', 'a', 's', 'd']:
        #             self.wheelchair_publish(key)
        #         elif key == '\x03':  # Catch Ctrl+C to exit the loop
        #             break
        # except rospy.ROSInterruptException:
        #     pass
        # finally:
        #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    rospy.init_node('wheelchair_keyboard_control', anonymous=True)
    # Create the publisher object and start the control loop
    pub = WheelchairControlPublisher()
    pub.run()
