#!/usr/bin/env python3

import sys
import rospy

from ee106s22.srv import VelHandler
from wheel_controller import WheelController


class WheelNode:
    def __init__(self):
        rospy.init_node("wheel_controller_node", anonymous=True)  # Init Wheel Controller Node
        rospy.Service("wheel_controller", VelHandler, self.tick)  # Init Wheel Controller Service

        self.wheel_controller = WheelController(topic="/cmd_vel")

    def tick(self, req):
        vel = [req.xVelLin, req.zVelAng]
        return self.wheel_controller.publish_velocity(velocities=vel)


def main(argv):
    try:
        print("Starting Wheel Controller")
        wheel_node = WheelNode()
        rospy.spin()

    except KeyboardInterrupt:
        print("Halting Wheel Controller")


if __name__ == "__main__":
    main(sys.argv)
