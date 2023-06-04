#!/usr/bin/env python3

import sys
import rospy

from ee106s22.srv import TFHandler
from transformation_handler import TransformationHandler


class TFNode:
    def __init__(self):
        rospy.init_node("tf_handler", anonymous=True)
        rospy.Service("tf_handler", TFHandler, self.tick)

        self.tf = TransformationHandler(target_frame='/left_limit', source_frame='/base_scan')

    def tick(self, _):
        self.tf.set_transformation_matrix()
        t_mtx = self.tf.get_transformation_matrix()

        return t_mtx[0, :], t_mtx[1, :], t_mtx[2, :], []


def main(argv):
    try:
        print("Starting TF Handler")
        tf_node = TFNode()
        rospy.spin()

    except KeyboardInterrupt:
        print("Halting TF Handler")


if __name__ == "__main__":
    main(sys.argv)
