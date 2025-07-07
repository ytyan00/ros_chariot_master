import rospy
import tf
import math
from geometry_msgs.msg import Twist
from pid_controller.pid import PID

class DifferentialDriveRobot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('differential_drive_controller')

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()

        # PID controllers for linear and angular control
        self.linear_pid = PID(kp=1.0, ki=0.01, kd=0.1)
        self.angular_pid = PID(kp=4.0, ki=0.02, kd=0.2)

        # Target position
        self.x_target = 2.0  # Example target x position
        self.y_target = 2.0  # Example target y position

        # Rate for control loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def update_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.x_current, self.y_current = trans[0], trans[1]
            self.yaw_current = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed, skipping this iteration")

    def control_loop(self):
        while not rospy.is_shutdown():
            self.update_position()

            # Compute errors
            dx = self.x_target - self.x_current
            dy = self.y_target - self.y_current
            distance_error = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_target - self.yaw_current)

            # Compute PID outputs
            linear_velocity = self.linear_pid.update(distance_error)
            angular_velocity = self.angular_pid.update(angle_error)

            # Create Twist message
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = angular_velocity

            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)

            # Check if the robot is close enough to the target
            if distance_error < 0.1:
                rospy.loginfo("Target reached!")
                break

            self.rate.sleep()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    try:
        robot = DifferentialDriveRobot()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass
