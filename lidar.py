import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class LiDAR:
    def __init__(self, filter_vars):
        self.scan_data = []
        self.angle_increment = 0
        self.angle_min = 0

        # Sieve boundaries - set desired range for sieve
        self.filter_angles = filter_vars  # Range of wanted LiDAR angles [start, stop]
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.callback)  # LiDAR Subscriber

    def callback(self, data):
        self.scan_data = data.ranges
        self.angle_increment = data.angle_increment
        self.angle_min = data.angle_min

    def range_sieve(self, t_mtx):
        filtered_distances = []
        front_dist = 0
        ray_angle = self.angle_min  # Start at minimum angle
        filter_start = self.filter_angles[0]  # Filter trigger point
        for it in range(len(self.scan_data)):
            r = self.scan_data[it]  # Grab range from ranges array

            distance = self.calculate_distance(r, ray_angle, t_mtx)  # Calculate distance from point

            # Check if front has reach destination
            if it == 0:
                front_dist = round(distance[0], 2)  # Set front distance

            # Break loop if angle is past desired range
            if ray_angle > self.filter_angles[1]:
                break

            # If filter start has been reached trigger sieve
            if ray_angle >= filter_start:
                filtered_distances.append(round(distance[1], 2))  # Transform point and record x

            ray_angle = self.angle_min + it * self.angle_increment

        return filtered_distances, front_dist

    def calculate_distance(self, r, ray_angle, t_mtx):
        if not str(r) == "inf":
            x = r * np.cos(ray_angle)
            y = r * np.sin(ray_angle)
        else:
            x = -1
            y = -1

        return np.dot(t_mtx, [x, y, 0, 1])
