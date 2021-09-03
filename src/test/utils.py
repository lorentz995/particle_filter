from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, \
    Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy
import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss
import rospy
import math
import time

import numpy as np
from numpy.random import random_sample


def test():
    odom = PoseWithCovarianceStamped()
    theta = 0
    while True:
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        # set the position
        x, y, theta = 5, 5, theta + 1
        time.sleep(0.1)
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, theta)
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


def convert_translation_rotation_to_pose(translation, rotation):
    """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
    return Pose(position=Point(x=translation[0], y=translation[1], z=translation[2]),
                orientation=Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]))


def convert_pose_inverse_transform(pose):
    """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
    translation = np.zeros((4, 1))
    translation[0] = -pose.position.x
    translation[1] = -pose.position.y
    translation[2] = -pose.position.z
    translation[3] = 1.0
    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler_angle = euler_from_quaternion(rotation)
    rotation = np.transpose(rotation_matrix(euler_angle[2], [0, 0, 1]))  # the angle is a yaw
    transformed_translation = rotation.dot(translation)

    translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
    rotation = quaternion_from_matrix(rotation)
    return (translation, rotation)


'''def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw'''


def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]


def sum_vectors(vectors):
    """Calculates the angle of the sum of vectors"""
    tot_vector = np.sum(vectors, axis=0)
    # sum vectors
    angle = math.atan2(tot_vector[1], tot_vector[0])
    # comes in radians for -pi to pi
    return math.degrees(angle) + 180


