#!/usr/bin/env python3

import sys
import rospy

from ee106s22.srv import StateMachineHandler
from state_machine import StateMachine


class SMNode:
    def __init__(self):
        rospy.init_node("state_machine_handler", anonymous=True)  # Init State Machine Node
        rospy.Service("state_machine", StateMachineHandler, self.tick)  # Init SM Service

        self.state_machine = StateMachine()

    def tick(self, req):
        self.state_machine.transition(min_dist=req.min_dist, front_dist=req.front_dist)
        action = self.state_machine.action()
        return action[0], action[1]


def main(argv):
    try:
        print("Starting SM Handler")
        sm_node = SMNode()
        rospy.spin()

    except KeyboardInterrupt:
        print("Halting SM Handler")


if __name__ == "__main__":
    main(sys.argv)
