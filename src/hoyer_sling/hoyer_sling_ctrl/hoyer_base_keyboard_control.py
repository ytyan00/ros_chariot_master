from hoyer_sling_base import hoyer_sling_base
import keyboard
import time

# Instantiate the HoyerSlingBase class
base = hoyer_sling_base()

# Start listening for keyboard inputss
print("Press 'w', 'a', 's', 'd' to control the base. Press 'x' to quit.")
linear_v = 75
angular_v = 55
try:
    while True:
        if keyboard.is_pressed('w'):
            base.move_with(linear_v, linear_v)  # Move forward
        elif keyboard.is_pressed('s'):
            print("True")
            base.move_with(-linear_v, -linear_v)  # Move backward
        elif keyboard.is_pressed('a'):
            base.move_with(-angular_v, angular_v)  # Turn left
        elif keyboard.is_pressed('d'):
            base.move_with(angular_v, -angular_v)  # Turn right
        elif keyboard.is_pressed('x'):
            print("Exiting...")
            base.move_with(0, 0)
            # break
        else:
            pass # Do nothing if no keys are pressed
        time.sleep(0.05)  # Add a slight delay to avoid high CPU usage
except KeyboardInterrupt:
    print("Program interrupted. Stopping base.")
    base.move_with(0, 0)