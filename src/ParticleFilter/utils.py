from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, \
    Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy
import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors


class Particle(object):
    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0),
                    orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2],
                                           w=orientation_tuple[3]))


class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
        obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
    """

    def __init__(self, map):
        self.map = map  # save this for later
        # build up a numpy array of the coordinates of each grid cell in the map
        X = np.zeros(
            (self.map.info.width * self.map.info.height, 2))  # qui c'era self.map.info.width*self.map.info.height, 2

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # build up a numpy array of the coordinates of each occupied grid cell in the map
        O = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    O[curr, 0] = float(i)
                    O[curr, 1] = float(j)
                    curr += 1

        # use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1, algorithm="ball_tree").fit(O)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = {}
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j * self.map.info.width
                self.closest_occ[ind] = distances[curr][0] * self.map.info.height
                curr += 1

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
            is out of the map boundaries, nan will be returned. """
        x_coord = int((x - self.map.info.origin.position.x / self.map.info.resolution))
        y_coord = int((y - self.map.info.origin.position.y / self.map.info.resolution))

        # check if we are in bounds
        if x_coord > self.map.info.width or x_coord < 0:
            return float('nan')
        if y_coord > self.map.info.height or y_coord < 0:
            return float('nan')

        ind = x_coord + y_coord * self.map.info.width
        if ind >= self.map.info.width * self.map.info.height or ind < 0:
            return float('nan')
        return self.closest_occ[ind]


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

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


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