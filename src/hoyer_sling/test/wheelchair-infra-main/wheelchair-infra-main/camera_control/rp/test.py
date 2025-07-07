#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            # Replace 'target_frame' and 'source_frame' with actual frame names
            (trans, rot) = listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
            print("Translation: ", trans)
            # print("Rotation: ", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
