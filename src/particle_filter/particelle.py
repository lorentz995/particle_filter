#!/usr/bin/env python3

import rospy
from octomap_msgs.srv import GetOctomap

from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss
from particle_filter.utils import Particle
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
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate
        self.odom_frame = "odom"
        self.scan_topic = "base_scan"
        self.n_particles = 1000
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)

        self.particle_cloud = []

        # self.current_odom_xy_theta = []

        self.initialized = True

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.map_frame),
                                            poses=particles_conv))

    def initialize_particle_cloud(self, map, xy_theta=(0.0, 0.0, 0.0)):
        # if xy_theta is None:
        # xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        rad_min = map.info.width * map.info.resolution / 2  # meters
        rad_max = map.info.height * map.info.resolution / 2
        self.particle_cloud = []
        self.particle_cloud.append(Particle(0.0, 0.0, 0.0))  # origine robot ??
        for i in range(self.n_particles - 1):
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            other_theta = random.random() * 360
            radius_min = random.random() * rad_min
            radius_max = random.random() * rad_max
            # x => straight ahead
            x = radius_min * math.sin(other_theta) + (
                    (map.info.width - map.info.origin.position.x) * map.info.resolution) / 2
            y = radius_max * math.cos(other_theta) + (
                    (map.info.height - map.info.origin.position.y) * map.info.resolution) / 2
            particle = Particle(x, y, theta)
            self.particle_cloud.append(particle)
        p.publish_particles(self.particle_cloud)

        return self.particle_cloud


def load_map(map):
    # occupancy_field = OccupancyField(map)
    print("second callback")


if __name__ == '__main__':
    try:
        p = ParticleFilter()
        time.sleep(0.3)
        rospy.Subscriber("/projected_map", OccupancyGrid, p.initialize_particle_cloud)
        # rospy.Subscriber("/projected_map", OccupancyGrid, load_map)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
