#!/usr/bin/env python3
import sys
import os

import rospy

import lidar
import transformation_handler

from geometry_msgs.msg import Twist
from ee106s22.srv import StateMachineHandler


class Turtlebot:
    def __init__(self):
        rospy.init_node('turtlebot3', anonymous=True)  # Init Turtlebot node
        self.rate = rospy.Rate(10)  # Rate

        #  LiDAR variables
        self.filter_angles = [1.52, 1.62]
        self.filtered_distances = []

        # Destination flag and min_dist
        self.is_at_dest = False
        self.min_dist = 0
        self.front_dist = 0

        # Necessary components
        self.vel = Twist()
        self.lidar = lidar.LiDAR(self.filter_angles)
        self.tf_handler = transformation_handler.TransformationHandler()
        self.sm_handler = rospy.ServiceProxy('state_machine', StateMachineHandler)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Twist Velocity Publisher

        self.run()  # Launch Turtlebot

    # Sets velocities to 0 and stops Turtlebot
    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)

    # Main controller loop for Turtlebot
    def run(self):
        while not self.is_at_dest:
            self.tf_handler.set_transformation_matrix()
            self.filtered_distances, self.front_dist = self.lidar.range_sieve(self.tf_handler.get_transformation_matrix())
            self.min_dist = min(self.filtered_distances)

            if 0 < self.front_dist < 0.30:
                self.is_at_dest = True

            os.system('clear')  # Clear terminal to keep output clean
            print(f"/base_scan -> /left_limit Transformation Matrix:")
            print(f"{self.tf_handler.get_transformation_matrix()}\n")

            print(f"Distances (m) for [{self.filter_angles[0]}, {self.filter_angles[1]}] rad:")
            print(f"{self.filtered_distances}\n")

            print(f"Min Distance: {self.min_dist} \nFront Distance: {self.front_dist}\n")

            print("Current velocity:")
            rospy.wait_for_service('state_machine')

            try:
                response_vel = self.sm_handler(self.min_dist, self.front_dist)

                print(f"Vx: {round(response_vel.lin_x, 2)}")
                print(f"Vyaw: {round(response_vel.ang_z, 2)}")

                self.vel.linear.x = response_vel.lin_x
                self.vel.angular.z = response_vel.ang_z

                self.vel_pub.publish(self.vel)

            except rospy.ServiceException as e:
                print(f"Service was unable to handle the request: {e}")

            self.rate.sleep()

        print("\n")
        print("Destination Reached!")
        print("Shutting down Turtlebot 3")
        self.stop()
        sys.exit()


def main(args):
    try:
        turtlebot = Turtlebot()
        rospy.spin()
    except KeyboardInterrupt:
        print("\n Shutting down Turtlebot 3")


if __name__ == '__main__':
    main(sys.argv)
