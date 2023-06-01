#!/usr/bin/env python3
import rospy
from ee106s22.msg import EE106lab_custom_new


def callback(data):
	rospy.loginfo(data.header.stamp)
	print(f"{data.something} + {data.somethingTwo} = " + str(data.something + data.somethingTwo))


def listener():
	rospy.init_node('listener')
	rospy.Subscriber('EE106lab_topic', EE106lab_custom_new, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
