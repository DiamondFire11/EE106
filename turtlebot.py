#!/usr/bin/env python3
import sys
import os

import rospy
import numpy as np

from ee106s22.srv import StateMachineHandler, TFHandler, LidarHandler, VelHandler


class Turtlebot:
    def __init__(self):
        rospy.init_node('turtlebot3', anonymous=True)
        self.rate = rospy.Rate(10)

        self.tf_handler = rospy.ServiceProxy("tf_handler", TFHandler)
        self.lidar_handler = rospy.ServiceProxy("lidar", LidarHandler)
        self.vel_handler = rospy.ServiceProxy("wheel_controller", VelHandler)
        self.sm_handler = rospy.ServiceProxy("state_machine", StateMachineHandler)

        # LiDAR vars
        self.filter_angles = [1.52, 1.62]
        self.left_ranges = []
        self.front_dist = 0
        self.min_dist = 0

        # Velocity vars
        self.vels = []

        self.run()

    def get_t_mtx(self):
        t_mtx = np.zeros([4, 4])

        rospy.wait_for_service('tf_handler')
        resp_mtx = self.tf_handler()

        t_mtx[0, :] = resp_mtx.rowA
        t_mtx[1, :] = resp_mtx.rowB
        t_mtx[2, :] = resp_mtx.rowC
        t_mtx[3, :] = [0, 0, 0, 1]

        return t_mtx

    def get_ranges(self):
        rospy.wait_for_service('lidar')
        resp_lidar = self.lidar_handler()

        if resp_lidar.success:
            self.left_ranges = list(resp_lidar.l_dist)
            for dist in range(len(self.left_ranges)):
                self.left_ranges[dist] = round(self.left_ranges[dist], 2)

            self.front_dist = resp_lidar.f_dist
            self.min_dist = min(self.left_ranges)

    def get_next_vel(self):
        rospy.wait_for_service("state_machine")
        resp_vel = self.sm_handler(self.min_dist, self.front_dist)
        self.vels = [resp_vel.lin_x, resp_vel.ang_z]

    def publish_vel(self):
        rospy.wait_for_service('wheel_controller')
        self.vel_handler(self.vels[0], self.vels[1])

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.get_ranges()
                self.get_next_vel()
                self.publish_vel()

                print(f"/base_scan -> /left_limit Transformation Matrix:")
                print(f"{self.get_t_mtx()}\n")

                print(f"Distances (m) for [{self.filter_angles[0]}, {self.filter_angles[1]}] rad:")
                print(f"{self.left_ranges}\n")

                print(f"Min Distance: {self.min_dist} \nFront Distance: {self.front_dist}\n")

                print("Current velocity:")
                print(f"Vx: {round(self.vels[0], 2)}")
                print(f"Vyaw: {round(self.vels[1], 2)}")

            except rospy.ServiceException as e:
                print(f"Service was unable to handle the request: {e}")


def main(args):
    try:
        turtlebot = Turtlebot()
        rospy.spin()
    except KeyboardInterrupt:
        print("\n Shutting down Turtlebot 3")


if __name__ == '__main__':
    main(sys.argv)
