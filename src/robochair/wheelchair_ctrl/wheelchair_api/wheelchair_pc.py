#!/usr/bin/env python3

import sys
sys.path.append('../..')
from udp_onboard_pc_to_rpi import RaspberryPiUDPClient

import time
import sys
import tty
import termios

class Wheelchair():
    def __init__(self):
        # Create a UDP Client for rpi communication
        self.client = RaspberryPiUDPClient(host='192.168.1.101', port=5005)
        time.sleep(1)
        print("[WHEELCHAIR] Initialzed")

    def cap_speed(self, speed):
        """
        Cap the input speed to be within the range [0, 100].

        Parameters:
            speed (float or int): The desired speed.

        Returns:
            int: The capped speed.
        """
        return max(0, min(100, int(speed)))


    def v_forward(self, speed):
        """
        Input: raw speed (0 - 100)\n

        Input range:
        - min fwd speed: 1
        - max fwd speed: 100
        - stop : 0\n

        The speed will be converted to hex on the Raspberry Pi
        """
        speed = self.cap_speed(speed)
        if speed > 0:
            command = f'F|{speed}'
            self.client.send_command(command)
        else:
            self.stop()
    
    def v_backward(self, speed):
        """
        Input: raw speed (0 - 100)\n

        Input range:
        - min fwd speed: 1
        - max fwd speed: 100
        - stop : 0\n

        Convert to wheelchair direction:
        - 1 -> 255
        - 100 -> 157
        - 0 -> 0

        The speed will be converted to hex on the Raspberry Pi
        """
        speed = self.cap_speed(speed)
        if speed > 0:
            speed = max(157, min(255, int(256 - speed)))
            command = f'B|{speed}'
            self.client.send_command(command)
        else:
            self.stop()
    
    def v_right(self, speed):
        """
        Input: raw speed (0 - 100)\n

        Input range:
        - min right speed: 1
        - max right speed: 100
        - stop : 0\n

        The speed will be converted to hex on the Raspberry Pi
        """
        speed = abs(speed)
        speed = self.cap_speed(speed)
        if speed > 0:
            command = f'R|{speed}'
            self.client.send_command(command)
        else:
            self.stop()

    def v_left(self, speed):
        """
        Input: raw speed (0 - 100)\n

        Input range:
        - min left speed: 1
        - max left speed: 100
        - stop : 0\n

        Convert to wheelchair direction:
        - 1 -> 255
        - 100 -> 157
        - 0 -> 0

        The speed will be converted to hex on the Raspberry Pi
        """
        speed = self.cap_speed(speed)
        if speed > 0:
            speed = max(157, min(255, int(256 - speed)))
            command = f'L|{speed}'
            self.client.send_command(command)
        else:
            self.stop()
    
    def stop(self):
        command = f'STOP'
        self.client.send_command(command)

    def press_key_to_continue(self, key="Enter"):
            """
            Stop the wheelchair, then wait for the user to press `key` before returning.
            By default, waits for Enter.
            """
            self.stop()
            prompt = f"Press {key} to move the wheelchair to the next point..."
            if key.lower() == "enter":
                input(prompt)
                return

            # For any other single-character key:
            print(prompt)
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while True:
                    ch = sys.stdin.read(1)
                    if ch.lower() == key.lower():
                        break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)