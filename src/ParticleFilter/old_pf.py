import time
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import math

def prova():
    odom = PoseWithCovarianceStamped()
    theta = 0
    while True:
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        # set the position
        x, y, theta = 5, 5, theta + 1
        time.sleep(0.1)
        print(theta, theta)
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, math.radians(theta))
        robot_pose = Pose(position=Point(x=x, y=y),
                          orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1],
                                                 z=orientation_tuple[2], w=orientation_tuple[3]))
        odom.pose.pose = robot_pose
        # set the covariance of the position

        odom.pose.covariance = [1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        pose_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('test')
    pose_pub = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped, queue_size=10)

    prova()
    rospy.spin()