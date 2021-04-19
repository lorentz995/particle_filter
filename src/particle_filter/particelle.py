#!/usr/bin/env python3

import rospy
from octomap_msgs.srv import GetOctomap

from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.srv import GetMap
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
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate
        self.odom_frame = "odom"
        self.scan_topic = "player0/scan"
        self.n_particles = 1000
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.tf_listener = TransformListener()
        self.particle_cloud = []
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.lidar_received)
        self.current_odom_xy_theta = []

        self.initialized = True

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.map_frame),
                                            poses=particles_conv))

    def lidar_received(self, msg):
        # listen to transform
        (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.odom_frame, rospy.Time(0))
        # print the transform
        rospy.loginfo('---------')
        rospy.loginfo('Translation: ' + str(trans))
        rospy.loginfo('Rotation: ' + str(rot))

        self.odom_pose = trans, rot
        # print(self.odom_pose)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose)
        update_initial_pose(new_odom_xy_theta)
        ''' Sta roba a che cazzo servirà?
        # calculate pose of laser relative ot the robot base
        
        p = PoseStamped(header=Header(stamp=rospy.Time.now(),
                                      frame_id=msg.header.frame_id))


        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
       
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)'''

    def initialize_particle_cloud(self, map, xy_theta=None):
        if xy_theta is None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        rad_min = map.info.width * map.info.resolution / 2  # meters
        rad_max = map.info.height * map.info.resolution / 2
        self.particle_cloud = []
        self.particle_cloud.append(Particle(xy_theta[0], xy_theta[1], xy_theta[2]))  # origine robot ??

        while len(self.particle_cloud) < self.n_particles:
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            # other_theta = random.random() * 360
            x = random.normalvariate((map.info.width - map.info.origin.position.x) * map.info.resolution / 2,
                                     rad_min / 3)  # * rad_min
            y = random.normalvariate((map.info.height - map.info.origin.position.y) * map.info.resolution / 2,
                                     rad_max / 3)  # * rad_max

            particle = Particle(x, y, theta)
            if 0 < (x / map.info.resolution) < map.info.width and 0 < (y / map.info.resolution) < map.info.height:
                self.particle_cloud.append(particle)
            else:
                print(x, y)

        pf.publish_particles(self.particle_cloud)

        return self.particle_cloud

    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation, rotation),
                        header=Header(stamp=msg.header.stamp, frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = convert_pose_to_xy_and_theta(msg.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(msg)


if __name__ == '__main__':
    try:
        pf = ParticleFilter()

        time.sleep(0.3)
        rospy.Subscriber("/projected_map", OccupancyGrid, pf.initialize_particle_cloud)
        # rospy.Subscriber("/projected_map", OccupancyGrid, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
