#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import select
import termios
import tty

class WheelchairTargetKeyboardPublisher:
    """
    Keyboard-based control to publish movement commands ('w', 'a', 's', 'd') to the /wheelchair_target topic.
    """

    def __init__(self):
        # Create a publisher for the /wheelchair_target topic
        self.pub = rospy.Publisher('/wheelchair_target', String, queue_size=10)
        rospy.init_node('wheelchair_keyboard_control', anonymous=True)
        rospy.loginfo("Control wheelchair using keyboard: 'w' - forward, 'a' - left, 's' - backward, 'd' - right")
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """
        Capture a single key press.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

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
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if key in ['w', 'a', 's', 'd']:
                    self.wheelchair_publish(key)
                elif key == '\x03':  # Catch Ctrl+C to exit the loop
                    break
        except rospy.ROSInterruptException:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    # Create the publisher object and start the control loop
    pub = WheelchairTargetKeyboardPublisher()
    pub.run()
