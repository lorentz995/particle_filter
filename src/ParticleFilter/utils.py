from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, \
    Pose, Point, Quaternion, PoseArray, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import rospy
import tf

from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

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


def printProgressBar(iteration, total, prefix='', suffix='', decimals=1, length=100, fill='', printEnd="\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=printEnd)
    # Print New Line on Complete
    if iteration % 100 == 0:
        print()
    return


def publish_particles(particle_pub, marker_pub, particle_cloud, frame):
    particles_conv = []
    for p in particle_cloud:
        particles_conv.append(p.as_pose())

    # actually send the message so that we can view it in rviz
    particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                 frame_id=frame),
                                   poses=particles_conv))

    marker_array = []
    for index, particle in enumerate(particle_cloud):
        marker = Marker(header=Header(stamp=rospy.Time.now(),
                                      frame_id=frame),
                        pose=particle.as_pose(),
                        type=0,
                        scale=Vector3(x=0.01, y=0.01, z=particle.w * 100),
                        id=index,
                        color=ColorRGBA(b=1, a=1))
        marker_array.append(marker)

    marker_pub.publish(MarkerArray(markers=marker_array))
    print("Published new particle cloud!")


def normalize_particles(particle_cloud):
    tot_weight = sum([particle.w for particle in particle_cloud]) or 1
    for particle in particle_cloud:
        particle.w = particle.w / tot_weight
    return particle_cloud


def pose_to_xytheta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]
