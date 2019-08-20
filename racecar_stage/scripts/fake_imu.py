#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class FakeImu:
  def __init__(self):
    rospy.init_node('fake_imu')

    self.pub = rospy.Publisher('/imu_data', Imu, latch=True, queue_size=1)
    self.sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

  def odom_cb(self, odom):
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "imu_link"
    imu.orientation = odom.pose.pose.orientation
    self.pub.publish(imu)

  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  imu = FakeImu()
  imu.spin()
