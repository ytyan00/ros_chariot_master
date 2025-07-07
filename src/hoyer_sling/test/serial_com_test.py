#!/usr/bin/env python3
import time
import serial
# import hoyer_sling
import hoyer_sling_frame
import rospy
import hoyer_sling_lift
# Set up the serial connection to the Arduino
# Use the appropriate port name for your system
# arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)


# # rospy.init_node("frame")
# # Allow time for the Arduino to reset after connecting
# time.sleep(2)

# try:
#     # Send the PWM command to move both motors with PWM = 55
#     arduino.write(b'PWM|-55|-55\n')
#     print("Sent: PWM|55|55")
    
#     # Keep the motors running for 3 seconds
#     time.sleep(2)
    
#     # Send the STOP command to stop both motors
#     arduino.write(b'STOP|1|1\n')
#     print("Sent: STOP|1|1")

# except KeyboardInterrupt:
#     # Close the serial connection on program exit
#     arduino.close()
#     print("Connection closed.")

# # Close the serial connection when done
# arduino.close()
# print("Motors stopped and connection closed.")

# robot = hoyer_sling.hoyer_sling()
# robot.base.start()
# robot.base.move_with(50,50)
# time.sleep(3)
# robot.base.stop()

# robot = hoyer_sling.hoyer_sling()

robot = hoyer_sling_frame.hoyer_sling_frame()
# time.sleep(3)
# print("move")
# start1 = time.time()
# while (time.time() -start1) < 2:
#     robot.base.move_with(-30,-30)
#     time.sleep(0.25)
# time.sleep(3)
# robot.base.move_with(0,0)
# time.sleep(2)
# print("fwd")
# start2 = time.time()
# while (time.time() -start2) < 2:
#     robot.base.move_with(30,30)
#     time.sleep(0.25)
# time.sleep(3)
# robot.base.stop()


print("frame")

def degree2teeth(x):
    return x * (4096/360)

robot.frame.set_dynamixel_position(int(robot.frame.get_dynamixel_position()+degree2teeth(10)))
time.sleep(10)
robot.frame.set_dynamixel_position(int(robot.frame.get_dynamixel_position()-degree2teeth(10)))
# time.sleep(3)
time.sleep(2)

# robot.lift.up(2000)
# time.sleep(2.1)
# robot.lift.stop()
# time.sleep(5)
# # print("lift")
# robot = hoyer_sling_lift.hoyer_sling_lift()
# robot.up(6000) #6000
# time.sleep(6)
# robot.stop()

# print("switch")
# time.sleep(1)
# # time.sleep(1)
# robot.up(4000)
# time.sleep(4.3)
# robot.stop()
# # time.sleep(5)
# # robot.lift.stop()

