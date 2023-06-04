import rospy
import tf
import numpy as np


class TransformationHandler:
    def __init__(self, target_frame='', source_frame=''):
        self.T = np.zeros([4, 4])
        self.tf_listener = tf.TransformListener()  # Transform Listener

        self.target_frame = target_frame
        self.source_frame = source_frame

    #  Get transformation matrix from ROS for /left_limit
    def set_transformation_matrix(self):
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
                break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.T = tf.transformations.quaternion_matrix(rot)
        self.T[0, 3] = trans[0]
        self.T[1, 3] = trans[1]
        self.T[2, 3] = trans[2]

    def get_transformation_matrix(self):
        return self.T
