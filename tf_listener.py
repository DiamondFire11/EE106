#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import LaserScan
roslib.load_manifest('ee106s22')

scan_data = []
angle_increment = 0
angle_min = 0


def callback(data):
    global scan_data, angle_increment, angle_min
    scan_data = data.ranges
    angle_increment = data.angle_increment
    angle_min = data.angle_min


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    # Subscribe to the /scan topic
    lidar_sub = rospy.Subscriber("/front/scan", LaserScan, callback)

    # initialization of the ROS tf listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    # the goal of this node is to continuously listen to the transformation relation between the base_link and
    # front_laser ROS frames and print the Translation and Rotation of the captured transformation matrix.
    while not rospy.is_shutdown():
        try:
            # capture the tf of the two frames the exact moment of the command execution (rospy.Time(0))
            (trans, rot) = listener.lookupTransform('/front_bumper', '/front_laser', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation
        # Matrix representation of the latter.
        print("Transform Data for /front_laser -> /front_bumper")
        print("The translation is (x,y,z) = " + str(trans))
        print("The rotation (quaternion) is (x,y,z,w) = " + str(rot))
        print("The rotation (euler) is (r,p,y) = " + str(tf.transformations.euler_from_quaternion(rot)))
        rot_mat = tf.transformations.quaternion_matrix(rot)
        print("The rotation (rotation matrix) is = " + str(tf.transformations.quaternion_matrix(rot)))

        # we assume that a Lidar point is detected, w.r.t the Lidar's frame
        detected_points = []
        for it in range(len(scan_data)):
            if np.isposinf(scan_data[it]) or np.isneginf(scan_data[it]):
                continue

            # Calculate ray ange
            ray_angle = angle_min + (it * angle_increment)

            # Extrapolate point position from ray angle
            x_pos = scan_data[it]*math.cos(ray_angle)
            y_pos = scan_data[it]*math.sin(ray_angle)
            z_pos = 0

            # Encode point data into 4D vector and append to points
            point = [x_pos, y_pos, z_pos, 1]
            detected_points.append(point)

        # initialization of the tf matrix to describe /front_laser in the /base_link frame
        rot_mat[0, 3] = trans[0]
        rot_mat[1, 3] = trans[1]
        rot_mat[2, 3] = trans[2]

        # Transform point data to the front_bumper frame and print
        for transformed_point in detected_points:
            bumper_point = np.dot(rot_mat, transformed_point)
            print(f"I see something at: \n X - {bumper_point[0]} \n Y - {bumper_point[1]} \n Z - {bumper_point[2]}")

        rate.sleep()
