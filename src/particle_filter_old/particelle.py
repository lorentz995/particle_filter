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
        self.height = 265
        self.width = 201  # o il contrario
        self.laser_pose = None
        self.odom_pose = None
        # self.robot_pose = None
        self.odom_to_map = None
        self.translation = None
        self.rotation = None
        self.occupancy_field = None
        self.MAX_RANGE = 3000
        self.RANGELIB_VAR = 3
        self.sigma = 0.08
        # motion model constants
        self.motion_variance_x = 0.05
        self.motion_variance_y = 0.025
        self.motion_variance_theta = 0.25
        # sensor model constants
        self.Z_SHORT = 0.01
        self.Z_MAX = 0.07
        self.Z_RAND = 0.12
        self.Z_HIT = 0.75
        self.SIGMA_HIT = 8.0

        self.lidar_angles_x = None
        self.lidar_angles_y = None
        self.tiled_angles = None
        self.first_sensor_update = False

        self.queries = None
        self.ranges = None
        self.sensor_model_table = None
        # Publishers
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.permissible_region = None
        self.map_initialized = False

        # Subscribers

        # self.odom_sub = rospy.Subscriber("player0/gps/odom", Odometry, self.odom)
        self.lidar_sub = rospy.Subscriber("/player0/scan_cloud", PointCloud2, self.scan_received)
        self.particle_cloud = []

        self.local_deltas = np.zeros((self.n_particles, 3))

        # Weights
        self.weights = np.ones(self.n_particles) / float(self.n_particles)
        self.initialized = True


        self.current_odom_xy_theta = []

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
        self.occupancy_field = OccupancyField(map)
        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map.data).reshape((map.info.height, map.info.width))

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        self.map_initialized = True

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
                if self.permissible_region[int(y / map.info.resolution)][int(x / map.info.resolution)]:
                    self.particle_cloud.append(particle)
            # else:
            # print(x, y)

        p.publish_particles(self.particle_cloud)
        self.normalize_particles()
        self.update_robot_pose()
        return self.particle_cloud

    def motion_model(self, proposal_dist, action):
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

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1
        for particle in self.particle_cloud:
            particle.w = particle.w / tot_weight


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
        # qua abbiamo robot_pose
    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, I hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """

        '''if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            print(msg.header.frame_id)
            print(msg.header.stamp)
            return
        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            print("2")
            return'''

        # calculate pose of laser relative ot the robot base
        ps = PoseStamped(header=Header(stamp=rospy.Time(0),
                                       frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, ps)

        # find out where the robot thinks it is based on its odometry
        ps = PoseStamped(header=Header(stamp=msg.header.stamp,
                                       frame_id=self.base_frame),
                         pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, ps)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)


        if not self.particle_cloud:
            print("ifnot")
            # now that we have all of the necessary transforms we can update the particle cloud
            # self.initialize_particle_cloud()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(msg)

        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            print("elif")
            self.update_particles_with_odom(msg)  # update based on odometry
            self.update_particles_with_laser(msg)  # update based on laser scan
            self.update_robot_pose()  # update robot's pose
            self.resample_particles()  # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)  # update map to odom transform now that we have new particles
            # publish particles (so things like rviz can see them)
        else: #TODO: cancella l'else, e controllare che venga diverso da 0
            print(math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]))
            print(math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]))
            print(math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]))
        self.publish_particles(msg)


    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation, rotation),
                        header=Header(stamp=msg.header.stamp, frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)

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

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        for particle in self.particle_cloud:
            tot_prob = 0
            for index, scan in enumerate(msg.ranges):
                x,y = self.transform_scan(particle,scan,index)
                # transform scan to view of the particle
                d = self.occupancy_field.get_closest_obstacle_distance(x,y)
                # calculate nearest distance to particle's scan (should be near 0 if it's on robot)
                tot_prob += math.exp((-d**2)/(2*self.sigma**2))
                # add probability (0 to 1) to total probability

            tot_prob = tot_prob/len(msg.ranges)
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

if __name__ == '__main__':
    try:
        p = ParticleFilter()
        time.sleep(0.3)
        rospy.Subscriber("/projected_map", OccupancyGrid, p.initialize_particle_cloud)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
