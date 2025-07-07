#!/usr/bin/env python3

import rospy
from dynamixel_sdk_examples.msg import SetPosition
from dynamixel_sdk_examples.srv import GetPosition, GetPositionRequest
import time
import sys
from fcntl import ioctl
import logging

# dynamixel MX64T interface for hoyer's frame 

 
class hoyer_sling_frame():
    def __init__(self):
        # Create a publisher object for the '/set_position' topic
        self.set_position_pub = rospy.Publisher('/set_position', SetPosition, queue_size=10)
        # Wait for a short time to ensure the publisher connection is established
        rospy.sleep(0.5)  # Wait 0.5 second for connections to establish

        
    def set_dynamixel_position(self, position, motor_id=1):
        """
        Set abs position of the motor (frame)\n
        position: abs position of the motor\n
        motor_id: by default is 1. If using multiple dynamixel motors change to the actual id
        """
        # Create a SetPosition message
        set_position_msg = SetPosition()
        set_position_msg.id = motor_id
        set_position_msg.position = position
        
        # Publish the message
        self.set_position_pub.publish(set_position_msg)
        rospy.loginfo(f"Position command sent to motor ID {motor_id}: {position}")

    def get_dynamixel_position(self, motor_id=1)->int:
        """
        Get the position of the motor, print in termial and return position as int
        """
        # Wait for the service to be available
        rospy.wait_for_service('/get_position')
        
        try:
            # Create a service proxy for the '/get_position' service
            get_position_service = rospy.ServiceProxy('/get_position', GetPosition)
            
            # Create a request object
            request = GetPositionRequest()
            request.id = motor_id
            
            # Call the service
            response = get_position_service(request)
            
            # Log the received position
            rospy.loginfo(f"Motor ID {motor_id} position: {response.position}")
            
            return int(response.position)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

        
if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('dynamixel_position_control', anonymous=True)

        hoyer_frame = hoyer_sling_frame()
        # Example usage: Set motor with ID 1 to position 1000
        hoyer_frame.set_dynamixel_position(motor_id=1, position=500)
        time.sleep(3)
        print(hoyer_frame.get_dynamixel_position())
    except rospy.ROSInterruptException:
        print("[WARM] Error to set posiiton.")