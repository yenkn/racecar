import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

yawPub = None
yawPosePub = None
imuMsg = None

yaw = None
firstYaw = None


def imu_callback(imu):
    global imuMsg, firstYaw, yaw
    imuMsg = imu
    (roll, pitch, yaw) = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
    data = Float64()
    data.data = yaw
    yawPub.publish(data)

    if firstYaw is None:
        firstYaw = yaw


def pose_callback(msg):
    if not imuMsg:
        return
    global yaw, firstYaw
    mypose = PoseStamped()
    mypose.header = msg.header
    mypose.pose.position = msg.pose.pose.position
    (x, y, z, w) = quaternion_from_euler(0, yaw - firstYaw, 0)
    mypose.pose.orientation.x = x
    mypose.pose.orientation.y = y
    mypose.pose.orientation.z = z
    mypose.pose.orientation.w = w
    yawPosePub.publish(mypose)


if __name__ == '__main__':
    rospy.init_node('get_yaw')
    yawPub = rospy.Publisher('/yaw_data', Float64, queue_size=1)
    yawPosePub = rospy.Publisher('/yaw_pose', PoseStamped, queue_size=1)
    rospy.Subscriber('/imu_data', Imu, imu_callback)
    rospy.Subscriber('/mrpt_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()
