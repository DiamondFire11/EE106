#!/usr/bin/env python3

import rospy
import random as rand
from ee106s22.msg import EE106lab_custom_new
from std_msgs.msg import Header


def talker():
	rospy.init_node('talker')
	pub = rospy.Publisher('EE106lab_topic', EE106lab_custom_new, queue_size=10)
	rate = rospy.Rate(10)  # 10Hz

	while not rospy.is_shutdown():
		header = Header()
		header.stamp = rospy.Time.now()
		ee106_message = EE106lab_custom_new()

		ee106_message.header = header
		ee106_message.something = rand.randint(1, 250)
		ee106_message.somethingTwo = rand.randint(1, 10)

		pub.publish(ee106_message)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
