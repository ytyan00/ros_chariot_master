import time
import serial
import sys
from fcntl import ioctl
import logging

 
 
class hoyer_sling_lift():
    def __init__(self):
        self.startup = False
        self.shutdown = None
        # Set up the serial connection to the Arduino
        # Use the appropriate port name for your system
        self.serial_wheel = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        
    def start(self):
        """
        [! NOT IMPLEMENTED !] Activate the lift for manpulation [! NOT IMPLEMENTED !]
        """
        self.serial_wheel.write(b'START|\n')

    def stop(self):
        """
        stop the lift
        """
        self.serial_wheel.write(b'STOP\n')
        time.sleep(0.25)

    def up(self,duration:int=1000):
        """
        lift up for duration (ms)
        """
        command = f'U|{duration}\n'
        self.serial_wheel.write(command.encode())
        time.sleep(duration/1000)

    def down(self,duration:int=1000):
        """
        lift down for duration (ms)
        """
        command = f'D|{duration}\n'
        self.serial_wheel.write(command.encode())
        time.sleep(duration/1000)
        
