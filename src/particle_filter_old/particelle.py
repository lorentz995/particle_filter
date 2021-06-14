#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from numpy.random.mtrand import normal
from sensor_msgs.msg import PointCloud2

from particle_filter_old.utils import *
from nav_msgs.msg import OccupancyGrid
import tf
import random
import time
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from ros_numpy import point_cloud2

import numpy as np
from numpy.random import random_sample


class ParticleFilter:
    def __init__(self):
        rospy.init_node('Particle_Filter')
        self.tf_listener = TransformListener()
        self.initialized = False
        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = math.pi / 6
        self.base_frame = "odom"  # the frame of the robot base
        self.map_frame = "odom"  # the name of the map coordinate
        self.odom_frame = "odom"
        self.scan_topic = "base_scan"
        self.n_particles = 1000

        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.permissible_region = None
        self.map_initialized = False

        # Subscribers

        self.laser = rospy.Subscriber("/player0/scan", LaserScan, self.laser_received)
        # self.lidar_sub = rospy.Subscriber("/player0/scan_cloud", PointCloud2, self.scan_received)
        self.particle_cloud = []

        self.local_deltas = np.zeros((self.n_particles, 3))

        # Weights
        self.weights = np.ones(self.n_particles) / float(self.n_particles)
        self.initialized = True

        self.current_odom_xy_theta = []

    def laser_received(self, msg):
        for index, scan in enumerate(msg.ranges):
            print("Laser: ", index, scan)

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.map_frame),
                                            poses=particles_conv))

    '''def motion_model(self, proposal_dist, action):
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
'''
    '''def measurement_model(self, proposal_dist, obs, weights, map):  # check num_rays, what is?
        num_rays = 46
        # TODO: che cazzo è num_rays
        # self.lidar_angles_x = np.linspace(-45, 45, 91)
        # self.lidar_angles_y = np.linspace(0, 45, 46)
        # self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)

        if self.first_sensor_update:
            self.queries = np.zeros((self.n_particles, 3), dtype=np.float32)
            self.ranges = np.zeros(num_rays * self.n_particles, dtype=np.float32)
            self.tiled_angles = np.tile(self.lidar_angles_x, self.n_particles)
            self.first_sensor_update = False

        self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
        self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
        self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
        self.queries[:, 2] += self.tiled_angles

        # compute the ranges for all the particles in a single functon call

        # resolve the sensor model by discretizing and indexing into the precomputed table
        obs /= float(map.info.resolution)
        ranges = self.ranges / float(map.info.resolution)

        intobs = np.rint(obs).astype(np.uint16)
        intrng = np.rint(ranges).astype(np.uint16)

        # compute the weight for each particle
        for i in range(self.n_particles):
            weight = np.product(self.sensor_model_table[intobs, intrng[i * num_rays:(i + 1) * num_rays]])
            # weight = np.power(weight, self.INV_SQUASH_FACTOR) eleva i pesi per inv_squash (quindi?)
            weights[i] = weight

    def odom(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        orientation = quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        print(pose)'''

    '''def precompute_measurement_model(self):
        z_short = self.Z_SHORT
        z_max = self.Z_MAX
        z_rand = self.Z_RAND
        z_hit = self.Z_HIT
        sigma_hit = self.SIGMA_HIT

        table_width = int(self.MAX_RANGE) + 1
        self.sensor_model_table = np.zeros((table_width, table_width))

        for d in range(table_width):  # controllare se scambiati
            norm = 0.0
            sum_unkown = 0.0
            # r is the observed range from the lidar unit
            for r in range(table_width):
                prob = 0.0
                z = float(r - d)
                # reflects from the intended object
                prob += z_hit * np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) / (sigma_hit * np.sqrt(2.0 * np.pi))

                # observed range is less than the predicted range - short reading
                if r < d:
                    prob += 2.0 * z_short * (d - r) / float(d)

                # erroneous max range measurement
                if int(r) == int(self.MAX_RANGE):
                    prob += z_max

                # random measurement
                if r < int(self.MAX_RANGE):
                    prob += z_rand * 1.0 / float(self.MAX_RANGE)

                norm += prob
                self.sensor_model_table[int(r), int(d)] = prob

            # normalize
            self.sensor_model_table[:, int(d)] /= norm

        # self.range_method.set_sensor_model(self.sensor_model_table) RANGE_METHOD da problemi'''

    def scan_received(self, msg):
        array = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=msg)
        for i in array:
            theta = math.atan2(i[1], i[0])
            d = math.sqrt(i[0]**2 + i[1]**2)
            print(theta, d)



if __name__ == '__main__':
    try:
        p = ParticleFilter()
        time.sleep(0.3)
        # rospy.Subscriber("/projected_map", OccupancyGrid, p.initialize_particle_cloud)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
