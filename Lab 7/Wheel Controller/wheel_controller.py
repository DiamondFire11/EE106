#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist


class WheelController:
    def __init__(self, topic=''):
        self.vel = Twist()
        self.vel_pub = rospy.Publisher(topic, Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def publish_velocity(self, velocities):
        try:
            self.vel.linear.x = velocities[0]
            self.vel.angular.z = velocities[1]

            self.vel_pub.publish(self.vel)
            self.rate.sleep()
            return True

        except rospy.ROSException:
            return False
