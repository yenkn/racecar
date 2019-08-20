import rospy
import time
from racecar_msgs.msg import ChassisCommand

if __name__ == '__main__':
    rospy.init_node('steer_test')

    motors_pub = rospy.Publisher(
        '/teleop/chassis_command', ChassisCommand, queue_size=1)

    raw_input()
    chassis_command = ChassisCommand()
    chassis_command.sender = 'teleop'
    chassis_command.header.stamp = rospy.Time.now()
    chassis_command.throttle = 0.1
    chassis_command.steering = 0.0
    motors_pub.publish(chassis_command)

    print 'start'

    raw_input()
    chassis_command.header.stamp = rospy.Time.now()
    chassis_command.throttle = 0.1
    chassis_command.steering = 0.5
    motors_pub.publish(chassis_command)
    print("left")

    # raw_input()
    # chassis_command.header.stamp = rospy.Time.now()
    # chassis_command.throttle = 0.12
    # chassis_command.steering = 0.0
    # motors_pub.publish(chassis_command)
    # print("middle")

    raw_input()
    chassis_command.header.stamp = rospy.Time.now()
    chassis_command.throttle = 0.1
    chassis_command.steering = -1.0
    motors_pub.publish(chassis_command)
    print("right")

    raw_input()
    chassis_command.header.stamp = rospy.Time.now()
    chassis_command.throttle = 0.0
    chassis_command.steering = 0.0
    motors_pub.publish(chassis_command)
    print("stop")
