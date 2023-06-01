#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi


class Turtlebot:
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()

    def run(self):
        vel = Twist()
        edges = [(267, 119), (273, 129), (280, 131), (280, 0)]

        for edge in edges:
            vel.linear.x = 0.15
            vel.angular.z = 0

            for i in range(edge[0]):
                self.vel_pub.publish(vel)
                self.rate.sleep()

            vel.linear.x = 0
            vel.angular.z = 0.15

            for i in range(edge[1]):
                self.vel_pub.publish(vel)
                self.rate.sleep()

        vel.linear.x = 0
        self.vel_pub.publish(vel)
        self.rate.sleep()
        

if __name__ == '__main__':
    try:
        tb = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
