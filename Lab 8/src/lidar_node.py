#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from ee106s23.srv import LidarHandler, TFHandler
from lidar import LiDAR


class LiDARNode:
    def __init__(self):
        rospy.init_node("lidar_handler", anonymous=True)    # Init Lidar Machine Node
        rospy.Service("lidar", LidarHandler, self.tick)     # Init LiDAR Service

        self.lidar = LiDAR(topic="/scan", filter_vars=[1.52, 1.62])

        # TF variables
        self.t_mtx = np.zeros((4, 4))
        self.init_t_mtx()
        self.tf_handler = rospy.ServiceProxy('tf_handler', TFHandler)

    def init_t_mtx(self):
        self.t_mtx = np.zeros((4, 4))
        self.t_mtx[3, :] = [0, 0, 0, 1]

    def tick(self, _):
        self.init_t_mtx()
        try:
            resp_mtx = self.tf_handler()

            self.t_mtx[0, :] = resp_mtx.rowA
            self.t_mtx[1, :] = resp_mtx.rowB
            self.t_mtx[2, :] = resp_mtx.rowC

        except rospy.ServiceException as e:
            print(f"Service was unable to handle the request: {e}")

        finally:
            front_dist = self.lidar.front_dist(t_mtx=self.t_mtx)
            left_ranges = self.lidar.range_sieve(t_mtx=self.t_mtx)
            return True, left_ranges, front_dist


def main(argv):
    try:
        print("Starting LiDAR Handler")
        lidar_node = LiDARNode()
        rospy.spin()

    except KeyboardInterrupt:
        print("Halting LiDAR Handler")


if __name__ == "__main__":
    main(sys.argv)
