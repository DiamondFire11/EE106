import rospy
import numpy as np

from sensor_msgs.msg import LaserScan


class LiDAR:
    def __init__(self, topic='', filter_vars=None):
        if filter_vars is None:
            filter_vars = [0, 6.2831]

        self.scan_data = []
        self.angle_increment = 0
        self.angle_min = 0

        # Sieve boundaries - set desired range for sieve
        self.filter_angles = filter_vars  # Range of wanted LiDAR angles [start, stop]
        self.lidar_sub = rospy.Subscriber(topic, LaserScan, self.callback)  # LiDAR Subscriber

    def callback(self, data):
        self.scan_data = data.ranges
        self.angle_increment = data.angle_increment
        self.angle_min = data.angle_min

    def range_sieve(self, t_mtx):
        filtered_distances = []
        ray_angle = self.angle_min  # Start at minimum angle
        filter_start = self.filter_angles[0]  # Filter trigger point

        for it in range(len(self.scan_data)):
            r = self.scan_data[it]  # Grab range from ranges array

            distance = self.calculate_distance(r, ray_angle, t_mtx)  # Calculate distance from point

            # Break loop if angle is past desired range
            if ray_angle > self.filter_angles[1]:
                break

            # If filter start has been reached trigger sieve
            if ray_angle >= filter_start:
                filtered_distances.append(round(distance[1], 2))  # Transform point and record x

            ray_angle = self.angle_min + it * self.angle_increment

        return filtered_distances

    def front_dist(self, t_mtx):
        distance = self.calculate_distance(self.scan_data[0], self.angle_min, t_mtx)
        return distance[0]

    def calculate_distance(self, r, ray_angle, t_mtx):
        if not str(r) == "inf" or float(r) == 0.0:
            x = r * np.cos(ray_angle)
            y = r * np.sin(ray_angle)
        else:
            x = 3.5
            y = 3.5

        return t_mtx @ [x, y, 0, 1]
