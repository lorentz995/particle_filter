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
from numpy.random import random_sample, normal


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
        self.sigma = 0.08

        self.particle_cloud = []

        self.current_odom_xy_theta = []

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
        self.lidar_sub = rospy.Subscriber("player0/scan_cloud", PointCloud2, self.lidar_received)
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

        self.occupancy_field = OccupancyField(map_msg)
        self.map_initialized = True

    def initial_pose(self, pose_msg):
        xy_theta = convert_pose_to_xy_and_theta(pose_msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(pose_msg)
        print("ho fatto initial pose")


    def initialize_particle_cloud(self, xy_theta=None):  # al posto di 0 0 0 c'è odom
        if xy_theta == None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)

        rad = 1  # meters

        self.particle_cloud = []
        self.particle_cloud.append(Particle(xy_theta[0], xy_theta[1], xy_theta[2]))
        for i in range(self.n_particles - 1):
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            other_theta = random.random() * 360
            radius = random.random() * rad
            # x => straight ahead
            x = radius * math.sin(other_theta) + xy_theta[0]
            y = radius * math.cos(other_theta) + xy_theta[1]
            particle = Particle(x, y, theta)
            self.particle_cloud.append(particle)

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
        print("lidar ricevuto")
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

        p = PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=lidar_msg.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=lidar_msg.header.stamp, frame_id=self.base_frame), pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(lidar_msg)
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > 0.2 or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > 0.2 or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > math.pi/6):
            # we have moved far enough to do an update!

            self.update_particles_with_odom(lidar_msg)    # update based on odometry
            self.update_particles_with_laser(lidar_msg)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(lidar_msg)     # update map to odom transform now that we have new particles
        else:
            print(math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]),
                  math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]),
                  math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]))

        # publish particles (so things like rviz can see them)
        # print("ho pubblicato le particelle")
        self.publish_particles(lidar_msg)

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        for particle in self.particle_cloud:
            r1 = math.atan2(delta[1], delta[0]) - old_odom_xy_theta[2]
            d = math.sqrt((delta[0]**2) + (delta[1]**2))

            particle.theta += r1 % 360
            particle.x += d * math.cos(particle.theta) + normal(0,0.1)
            particle.y += d * math.sin(particle.theta) + normal(0,0.1)
            particle.theta += (delta[2] - r1 + normal(0,0.1)) % 360

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        # print(msg.data)
        for particle in self.particle_cloud:
            tot_prob = 0
            for index, scan in enumerate(msg.data):
                x, y = self.transform_scan(particle, scan, index)
                # transform scan to view of the particle
                d = self.occupancy_field.get_closest_obstacle_distance(x, y)
                # calculate nearest distance to particle's scan (should be near 0 if it's on robot)
                tot_prob += math.exp((-d ** 2) / (2 * self.sigma ** 2))
                # add probability (0 to 1) to total probability

            tot_prob = tot_prob / len(msg.data)
            # normalize total probability back to 0-1
            particle.w = tot_prob
            # assign particles weight


    def transform_scan(self, particle, distance, theta):
        """ Calculates the x and y of a scan from a given particle
        particle: Particle object
        distance: scan distance (from ranges)
        theta: scan angle (range index)
        """
        return (particle.x + distance * math.cos(math.radians(particle.theta + theta)),
                particle.y + distance * math.sin(math.radians(particle.theta + theta)))

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        newParticles = []
        for i in range(len(self.particle_cloud)):
            # resample the same # of particles
            choice = random_sample()
            # all the particle weights sum to 1
            csum = 0 # cumulative sum
            for particle in self.particle_cloud:
                csum += particle.w
                if csum >= choice:
                    # if the random choice fell within the particle's weight
                    newParticles.append(deepcopy(particle))
                    break
        self.particle_cloud = newParticles

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
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation, rotation),
                        header=Header(frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)


if __name__ == '__main__':
    try:
        pf = ParticleFilter()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
