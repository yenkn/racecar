#!/usr/bin/env python

import rospy

from racecar_msgs.msg import ChassisCommand
from geometry_msgs.msg import Twist

class CmdVelTranslator:
    def __init__(self):
        rospy.init_node('cmd_vel_translator')

        self.max_speed = rospy.get_param('racecar_properties.max_speed', 10.0)
        self.max_steering = rospy.get_param('racecar_properties.max_steering', 0.7)

        self.pub = rospy.Publisher('controller/chassis_command', ChassisCommand, latch=True, queue_size=1)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

    def cmd_vel_cb(self, cmd_vel_data):
        command = ChassisCommand()
        command.header.stamp = rospy.Time.now()
        command.sender = 'controller'
        command.throttle = cmd_vel_data.linear.x / self.max_speed
        command.steering = cmd_vel_data.angular.z / self.max_steering
        self.pub.publish(command)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    translator = CmdVelTranslator()
    translator.spin()
