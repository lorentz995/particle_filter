#!/usr/bin/env python3

import rospy
from octomap_msgs.srv import GetOctomap

from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss
from particle_filter.utils import *
from nav_msgs.msg import OccupancyGrid

import math
import random
import time

import numpy as np
from numpy.random import random_sample, normal


class ParticleFilter:
    def __init__(self):
        self.initialized = False
        rospy.init_node('Particle_Filter')
        self.base_frame = "odom"  # the frame of the robot base
        self.map_frame = "odom"  # the name of the map coordinate
        self.odom_frame = "odom"
        self.scan_topic = "base_scan"
        self.n_particles = 1000

        # motion model constants
        self.motion_variance_x = 0.05
        self.motion_variance_y = 0.025
        self.motion_variance_theta = 0.25
        # Publishers
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)

        # Subscribers

        self.odom_sub = rospy.Subscriber("player0/gps/odom", Odometry, self.odom)
        self.particle_cloud = []

        # self.current_odom_xy_theta = []

        self.local_deltas = np.zeros((self.n_particles, 3))

        self.initialized = True

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.map_frame),
                                            poses=particles_conv))

    def initialize_particle_cloud(self, map, xy_theta=(0.0, 0.0, 0.0)):  # al posto di 0 0 0 c'è odom
        # if xy_theta is None:
        # xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        rad_min = map.info.width * map.info.resolution / 2  # meters
        rad_max = map.info.height * map.info.resolution / 2

        self.particle_cloud = []
        # self.particle_cloud.append(Particle(0.0, 0.0, 0.0))  # origine robot ??

        while len(self.particle_cloud) < self.n_particles:
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            # other_theta = random.random() * 360
            radius_min = random.normalvariate(0, 0.5) * rad_min
            radius_max = random.normalvariate(0, 0.5) * rad_max

            x = radius_min + ((map.info.width - map.info.origin.position.x) * map.info.resolution) / 2
            y = radius_max + ((map.info.height - map.info.origin.position.y) * map.info.resolution) / 2
            particle = Particle(x, y, theta)
            if 0 < (x / map.info.resolution) < map.info.width and 0 < (y / map.info.resolution) < map.info.height:
                self.particle_cloud.append(particle)
            # else:
            # print(x, y)

        p.publish_particles(self.particle_cloud)
        return self.particle_cloud

    def motion_model(self, proposal_dist, action):  # sostituire action con dim ?

        # rotate the action into the coordinate space of each particle
        # t1 = time.time()
        cosines = np.cos(proposal_dist[:, 2])
        sines = np.sin(proposal_dist[:, 2])

        self.local_deltas[:, 0] = cosines * action[0] - sines * action[1]
        self.local_deltas[:, 1] = sines * action[0] + cosines * action[1]
        self.local_deltas[:, 2] = action[2]

        proposal_dist[:, :] += self.local_deltas
        proposal_dist[:, 0] += np.random.normal(loc=0.0, scale=self.motion_variance_x, size=self.n_particles)
        proposal_dist[:, 1] += np.random.normal(loc=0.0, scale=self.motion_variance_y, size=self.n_particles)
        proposal_dist[:, 2] += np.random.normal(loc=0.0, scale=self.motion_variance_theta, size=self.n_particles)

    def odom(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        orientation = quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        print(pose)

if __name__ == '__main__':
    try:
        p = ParticleFilter()
        time.sleep(0.3)
        rospy.Subscriber("/projected_map", OccupancyGrid, p.initialize_particle_cloud)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
