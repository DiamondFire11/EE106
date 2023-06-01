#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class LiDARSense:
    def __init__(self):
        rospy.init_node("jackal_range_tracker", anonymous=True)  # Init ROS Node
        self.lidar_sub = rospy.Subscriber("/front/scan", LaserScan, self.callback)  # Subscribe to /front/scan
        self.tracker_pub = rospy.Publisher("/jackal_robot_status", String, queue_size=10)  # Publish status messages
        self.rate = rospy.Rate(10)  # Update at 10 Hz

    def callback(self, data):
        self.check_jackal_safety(data.ranges)
        self.rate.sleep()

    def check_jackal_safety(self, data):
        status = [False, False, True]
        for distance in data:
            if distance < 0.2:
                status = [True, status[1], False]

            if 0.2 <= distance < 0.5:
                status = [status[0], True, False]

            if distance <= 1.0:
                status = [status[0], status[1], False]

        self.tracker_pub.publish(f"Critical: {status[0]} - Major: {status[1]} - Minor: {status[2]}")


def main():
    try:
        LiDARSense()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
