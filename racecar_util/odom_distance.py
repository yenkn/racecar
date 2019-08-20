import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

lastTime = None
distance = 0.0
disPub_ = None

def speedCallback(msg):
    global lastTime, distance
    speed = msg.linear.x
    now = rospy.get_rostime().now().to_sec()
    if not lastTime:
        lastTime = now
        return
    distance += (now - lastTime) * speed
    lastTime = now
    msg = Float32()
    msg.data = distance
    disPub_.publish(msg)

if __name__ == '__main__':
    rospy.init_node('odom_distance', anonymous=True)
    rospy.Subscriber('/speed', Twist, speedCallback)
    disPub_ = rospy.Publisher('/distance', Float32, queue_size=1)
    rospy.spin()
