import rospy
from nav_msgs.msg import Path
from racecar_msgs.srv import GetPathSimilarity
import math

lastPath = None

def path_compare_client(path1, path2):
    rospy.wait_for_service('compare_path')
    try:
        path_compare = rospy.ServiceProxy('compare_path', GetPathSimilarity)
        resp1 = path_compare(path1, path2)
        return resp1.similarity
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def pathCallback(path):
    global lastPath
    if lastPath is None:
        lastPath = path
        return

    start = rospy.get_rostime().to_sec()
    print "similarity: %f\t time: %f" % (path_compare_client(path, lastPath), rospy.get_rostime().to_sec() - start)
    lastPath = path


if __name__ == '__main__':
    rospy.init_node('path_change')
    pathSub = rospy.Subscriber('/scan_planner/plan', Path, pathCallback, queue_size=1)
    rospy.spin()
