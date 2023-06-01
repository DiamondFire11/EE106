#!/usr/bin/env python3

import sys
import rospy

from ee106s22.srv import StateMachineHandler
from state_machine import StateMachine


class SMNode:
    def __init__(self):
        self.state_machine = StateMachine()

        rospy.init_node("state_machine", anonymous=True)  # Init State Machine Node
        rospy.Service('state_machine', StateMachineHandler, self.tick)  # Init SM Service

    def tick(self, req):
        self.state_machine.transition(min_dist=req.min_dist)
        return self.state_machine.action()


def main(argv):
    try:
        print("Starting SM Handler")
        sm_node = SMNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("Halting SM Handler")


if __name__ == "__main__":
    main(sys.argv)
