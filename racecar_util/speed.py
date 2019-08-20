import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

filterPub = None
lastSpeed = 0


def speedCallback(twist):
    global filterPub, lastSpeed
    val = Float64()
    val.data = twist.linear.x - lastSpeed
    lastSpeed = twist.linear.x
    filterPub.publish(val)


if __name__ == '__main__':
    rospy.init_node("speed_filter")
    filterPub = rospy.Publisher("/filter", Float64, queue_size=1)
    speedSub = rospy.Subscriber("/speed", Twist, speedCallback, queue_size=1)
    rospy.spin()
