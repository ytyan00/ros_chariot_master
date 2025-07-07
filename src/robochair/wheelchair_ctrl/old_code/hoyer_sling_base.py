import time
import serial
import sys
from fcntl import ioctl
import logging

 
 
# hoyer sling base orientation
#
#  |                  |
#  |                  |
#  |                  |
#  |                  |
#  |                  |     
# []        ^         []
# []<-left--|--right->[]
# []                  []



class hoyer_sling_base():
    def __init__(self):
        self.drive_mode = True # True means drive mode. False means setting mode
        self.startup = False
        self.shutdown = None
        # Set up the serial connection to the Arduino
        # Use the appropriate port name for your system
        self.serial_wheel = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        
    def start(self):
        """
        activate the base for manpulation
        """
        self.serial_wheel.write(b'START|\n')

    def stop(self):
        """
        stop the base
        """
        self.serial_wheel.write(b'STOP|1|1\n')
        time.sleep(0.2)

    def move_with(self, left_PWM, right_PWM):
        command = f'PWM|{left_PWM}|{right_PWM}\n'
        self.serial_wheel.write(command.encode())

    def wheel_brake(self,left_brake:bool,right_brake:bool) -> None:
        """
        inputs: \n
        left()right_brake (bool): True for ON, False for OFF 
        """
        if left_brake and right_brake:
            self.stop()
        else:
            if left_brake:
                self.serial_wheel.write(b'STOP|0|1\n')
            if right_brake:
                self.serial_wheel.write(b'STOP|0|1\n')
        
