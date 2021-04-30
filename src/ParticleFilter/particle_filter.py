#!/usr/bin/env python3
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros as tf2
from ParticleFilter.utils import *
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import random
import time

import numpy as np
from numpy.random import random_sample


class ParticleFilter:
    def __init__(self):
        self.initialized = False
        rospy.init_node('Particle_Filter')

        # Viz parameters
        self.base_frame = "base_link"  # the frame of the robot base
        # self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        # self.scan_topic = "base_scan"  # the topic where we will get laser scans from

        self.n_particles = 500  # the number of particles to use

        # Map parameters
        self.map_initialized = False
        self.map_width, self.map_height = None, None
        self.map_resolution = None
        self.map_origin_x, self.map_origin_y, self.map_origin_z = None, None, None
        self.permissible_region = None

        # Setup publishers and subscribers
        self.map_listener = rospy.Subscriber("projected_map", OccupancyGrid, self.get_octomap)
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose)
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        # self.lidar_sub = rospy.Subscriber("player0/scan_cloud", PointCloud2, self.lidar_received)
        # Setup listener and broadcaster for tf_messages
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        # riceve in ingresso il click da rviz di PoseEstimate2D, da sostituire con valori discreti
        # time.sleep(1)
        # Ci vuole un po' di tempo per eseguire la callback, usare i controlli initialized

    def get_octomap(self, map_msg):
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_resolution = map_msg.info.resolution
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        # self.map_origin_z = map_msg.info.origin.position.z # non serve al momento

        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        self.map_initialized = True

    def initial_pose(self, pose_msg):
        xy_theta = convert_pose_to_xy_and_theta(pose_msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(pose_msg)
        print("ho fatto initial pose")


    def initialize_particle_cloud(self, xy_theta=(0.0, 0.0, 0.0)):  # al posto di 0 0 0 c'è odom

        # if xy_theta is None:
        # xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)

        rad_min = self.map_width * self.map_resolution / 2  # meters
        rad_max = self.map_width * self.map_resolution / 2

        self.particle_cloud = []
        # self.particle_cloud.append(Particle(0.0, 0.0, 0.0))  # origine robot ??

        while len(self.particle_cloud) < self.n_particles:
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            # other_theta = random.random() * 360
            radius_min = random.normalvariate(0, 0.5) * rad_min
            radius_max = random.normalvariate(0, 0.5) * rad_max

            x = radius_min + ((self.map_width - self.map_origin_x) * self.map_resolution) / 2
            y = radius_max + ((self.map_height - self.map_origin_y) * self.map_resolution) / 2
            particle = Particle(x, y, theta)
            if 0 < (x / self.map_resolution) < self.map_width and 0 < (y / self.map_resolution) < self.map_height:
                if self.permissible_region[int(y / self.map_resolution)][int(x / self.map_resolution)]:
                    self.particle_cloud.append(particle)

            # else:
            # print(x, y)

        self.publish_particles(self.particle_cloud)
        print("ho pubblicato le particelle")
        self.normalize_particles()
        self.update_robot_pose()

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.odom_frame),
                                            poses=particles_conv))

    def lidar_received(self, lidar_msg):
        if not (self.tf_listener.canTransform(self.base_frame, lidar_msg.header.frame_id, lidar_msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            print("base transform error")

            return

        if not (self.tf_listener.canTransform(self.base_frame, self.odom_frame, lidar_msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            # print("odom.pose error")
            print("odom error")
            return

        # calculate pose of laser relative ot the robot base
        print("ok")
        try:
            p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                          frame_id=lidar_msg.header.frame_id))
            print(p)
            self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

            # find out where the robot thinks it is based on its odometry
            p = PoseStamped(header=Header(stamp=lidar_msg.header.stamp,
                                          frame_id=self.base_frame),
                            pose=Pose())
            self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
            # store the the odometry pose in a more convenient format (x,y,theta)
            # new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        except:
            print(p)
            pass

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1
        for particle in self.particle_cloud:
            particle.w = particle.w / tot_weight
        print("ho normalizzato le particelle")

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            Computed by taking the weighted average of poses.
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        x = 0
        y = 0
        theta = 0
        angles = []
        for particle in self.particle_cloud:
            x += particle.x * particle.w
            y += particle.y * particle.w
            v = [particle.w * math.cos(math.radians(particle.theta)),
                 particle.w * math.sin(math.radians(particle.theta))]
            angles.append(v)
        theta = sum_vectors(angles)
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.robot_pose = Pose(position=Point(x=x, y=y),
                               orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1],
                                                      z=orientation_tuple[2], w=orientation_tuple[3]))
        print("update intial pose")


    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer """
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(self.translation, self.rotation),
                        header=Header(stamp=msg.header.stamp, frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)


if __name__ == '__main__':
    try:
        pf = ParticleFilter()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
