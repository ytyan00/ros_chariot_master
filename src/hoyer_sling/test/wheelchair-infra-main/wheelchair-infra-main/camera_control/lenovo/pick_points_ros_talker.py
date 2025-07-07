#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from pick_point_helper import get_distance
import threading


value = Point()
update = False

def update_value():
    global value, update
    while not rospy.is_shutdown() and update == False:
        # getting a new value (e.g., from a sensor or calculation)
        x, y,z = get_distance()
        # x ,y position in camera coordinate, we assume that camera coordinate 
        # is the same as world coordinate
        value = Point(x- 0.2, y - 0.2 , z - 0.2)
        update = True

def main():
    rospy.init_node('publisher_node')
    
    # Create a publisher for the "target" topic, publishing Float32 messages
    target_publisher = rospy.Publisher('target', Point, queue_size=1)
    
    rate = rospy.Rate(5)  # Set the publishing rate (1 Hz in this example)
    

    # Create a thread to continuously update the value
    update_thread = threading.Thread(target=update_value)
    update_thread.start()

    while not rospy.is_shutdown():
        # Publish the value to the "target" topic
        
        target_publisher.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
