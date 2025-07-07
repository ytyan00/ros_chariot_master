import time
import serial
import hoyer_sling

# Set up the serial connection to the Arduino
# Use the appropriate port name for your system
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Allow time for the Arduino to reset after connecting
time.sleep(2)

# try:
#     # Send the PWM command to move both motors with PWM = 55
#     arduino.write(b'PWM|55|-55\n')
#     print("Sent: PWM|55|55")
    
#     # Keep the motors running for 3 seconds
#     time.sleep(3)
    
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

robot = hoyer_sling.hoyer_sling()
robot.lift.up(50000)
time.sleep(50)
# robot.lift.stop()
# print("switch")
# time.sleep(1)
# robot.lift.down(5000)
# time.sleep(5)
# robot.lift.stop()

