#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import threading
from python_interface import wheelchair
import logging

def print_helper():
    message = "Hi, welcome! To control the wheelchair, you can press:\n"
    message += " - 'w' to move forward\n"
    message += " - 's' to move backward\n"
    message += " - 'a' to turn left\n"
    message += " - 'd' to turn right\n"
    message += "If you want to set a different speed, you can press:\n"
    message += " - 'k' to increase speed\n"
    message += " - 'j' to decrease speed\n"
    message += "If you want make some noise, you can press:\n"
    message += " - 'b' for a short beep\n"
    message += " - 'r' for a longer beep\n"
    print(message)

def on_press_callback(command):
    """
    Callback function to process the command received from the /wheelchair_target topic.
    """
    if command == "k":
        wheelchair.increase_speed()
    elif command == "j":
        wheelchair.decrease_speed()
    elif command == 't':
        wheelchair.enter_setting_mode()
    elif command == 'y':
        wheelchair.enter_drive_mode()
    elif command == 'u':
        wheelchair.next_setting()

    elif command == 'b':
        wheelchair.shortBeep()

    elif command == 'r':
        wheelchair.playsong()

def wheelchair_control_callback(msg):
    """
    Callback function for the /wheelchair_target topic. It replaces the keyboard input.
    """
    command = msg.data
    rospy.loginfo(f"Received command: {command}")
    on_press_callback(command)  # Handle non-movement commands
    global x, y  # Global variables to store direction

    if command == "w":
        y = 100  # Move forward
    elif command == "s":
        y = 180  # Move backward
    else:
        y = 0  # Stop

    if command == "a":
        x = 180  # Turn left
    elif command == "d":
        x = 100  # Turn right
    else:
        x = 0  # Go straight

def always_move(wheelchair):
    """
    Function that continuously moves the wheelchair.
    """
    while not rospy.is_shutdown():
        wheelchair.move()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('wheelchair_target_subscriber', anonymous=True)

    # Create wheelchair object
    wheelchair = wheelchair.Wheelchair()
    wheelchair.start_steaming()
    print("Wheelchair created")

    # Initialize direction variables
    x, y = 0, 0

    # Subscribe to the /wheelchair_target topic
    rospy.Subscriber("/wheelchair_target", String, wheelchair_control_callback)

    # Thread to keep the wheelchair always moving
    move_thread = threading.Thread(target=always_move, args=(wheelchair,))
    move_thread.start()

    rospy.loginfo("Waiting for commands...")
    
    # Main loop for setting the direction and moving the wheelchair
    try:
        while not rospy.is_shutdown():
            wheelchair.set_direction(x, y)
            rospy.sleep(0.1)  # Update rate (10Hz)
    except rospy.ROSInterruptException:
        pass
