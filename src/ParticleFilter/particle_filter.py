#!/usr/bin/env python3
from ParticleFilter.utils import *
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Twist, Vector3, PoseStamped
from std_msgs.msg import Header
from ros_numpy import point_cloud2
import numpy as np
from numpy.random import random_sample, normal
import random


class ParticleFilter:
    def __init__(self):
        self.initialized = False
        rospy.init_node('Particle_Filter')
        # Viz parameters
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = 'map'
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.path = Path()
        # Parameters
        self.n_particles = 500  # the number of particles to use
        self.sigma_measurement = 0.03
        self.sigma_motion_model = 0.07
        self.sigma_likelihood = 0.03

        self.normalizer = 0

        self.particle_cloud = []
        self.current_odom_xy_theta = []
        self.robot_trajectory = []

        # Setup publishers and subscribers
        self.map_sub = rospy.Subscriber("projected_map", OccupancyGrid, self.get_map)
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.stoprobot = rospy.Publisher("player0/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()
        self.gps = rospy.Subscriber("player0/gps/odom", Odometry, self.odom_received)
        self.lidar_sub = rospy.Subscriber("player0/scan_cloud", PointCloud2, self.lidar_scan)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.current_pose = None
        self.tf_listener = TransformListener()
        self.initialized = True
        print("main inizializzato")

    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        self.stoprobot.publish(self.vel_msg)

    def get_map(self, map_msg):
        self.map = map_msg
        self.map_resolution = map_msg.info.resolution
        self.map_width = map_msg.info.width
        self.map_heigth = map_msg.info.height
        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1

        # self.landmarks = Landmarks(self.map)
        self.map_initializated = True
        self.likelihood = LikelihoodField(map_msg)
        print("mappa inizializzata")

    def initialize_particle_cloud(self, map):
        while len(self.particle_cloud) < self.n_particles:
            x = random.uniform(0, map.info.width * map.info.resolution)
            # x = random.uniform(0, 1)
            # y = random.uniform(0, 1)
            y = random.uniform(0, map.info.height * map.info.resolution)
            theta = random.random() * 360
            if self.permissible_region[int(y / map.info.resolution)][int(x / map.info.resolution)]:
                particle = Particle(x, y, theta)
                self.particle_cloud.append(particle)
        time.sleep(2)  # da sistemare in un qualche modo
        self.normalize()
        self.publish_particles()

    def normalize(self):
        tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1
        for particle in self.particle_cloud:
            particle.w = particle.w / tot_weight

    def publish_particles(self):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id=self.odom_frame),
                                            poses=particles_conv))

    def odom_received(self, odom_scan):
        new_odom_xy_theta = pose_to_xytheta(odom_scan.pose.pose)
        if self.current_odom_xy_theta:

            control_action = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                              new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                              new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        if (abs(control_action[0]) > 0 or
                abs(control_action[1]) > 0 or
                abs(control_action[2]) > 0):
            self.move_particles(control_action)

            self.move = True
        else:
            self.move = False

    def lidar_scan(self, lidar_msg):
        a = 0
        print("Lidar scan received..")
        if not self.initialized:
            print("Loading...")
            return
        if not self.particle_cloud:
            self.initialize_particle_cloud(self.map)
            print("Generated random particles")

        elif self.move:
            # self.stop()
            print("me so mosso")
            xyz_cloud = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=lidar_msg)
            if len(xyz_cloud) > 0:

                # Measurement model
                self.normalizer = 0
                for particle in self.particle_cloud:
                    a += (100 / self.n_particles)
                    print("Processing: ", a, "%")
                    w = 0

                    for index, scan in enumerate(xyz_cloud):
                        x1, y1 = scan[0] + normal(0, self.sigma_measurement), scan[1] + normal(0,
                                                                                               self.sigma_measurement)
                        distance = math.sqrt(x1 ** 2 + y1 ** 2)
                        angle = int(math.atan2(y1, x1) * (180 / math.pi))

                        xpart, ypart = self.transform_scan(particle, distance, angle)

                        # transform scan to view of the particle
                        d = self.likelihood.get_closest_obstacle_distance(xpart, ypart)

                        if math.isnan(d):
                            particle.w = 0.0
                        else:
                            wk = 1 / (math.sqrt(2 * math.pi) * self.sigma_likelihood) * math.exp(
                                -(d ** 2) / (2 * (self.sigma_likelihood ** 2)))

                            w += wk
                            # w += math.exp((-d ** 2) / (2 * self.sigma ** 2))

                    particle.w = (w / len(xyz_cloud))
                    x_coord = int(particle.x / self.map_resolution)
                    y_coord = int(particle.y / self.map_resolution)
                    if x_coord >= self.map_width or y_coord >= self.map_heigth:
                        particle.w = 0.0
                    elif not self.permissible_region[y_coord][x_coord]:
                        particle.w = 0.0
                    self.normalizer += particle.w
                # self.resample_particles()
                self.low_variance_resample()
                print("ricampiono le particelle")

    def transform_scan(self, particle, distance, angle):

        return (particle.x + distance * math.cos(math.radians(particle.theta + angle)),
                particle.y + distance * math.sin(math.radians(particle.theta + angle)))

    def resample_particles(self):
        # self.normalize()
        newParticles = []
        for i in range(len(self.particle_cloud)):
            chance = random_sample() * self.normalizer
            j = -1
            s = 0.0
            while (s < chance) and (j < self.n_particles):
                j += 1
                s += self.particle_cloud[j].w
            newParticles.append(deepcopy(self.particle_cloud[j]))

        self.particle_cloud = newParticles
        self.publish_particles()
        self.update_robot_pose()
        print("pubblicato le particelle")

    def low_variance_resample(self):
        newParticleCloud = []
        Minv = 1 / self.n_particles
        chance = Minv * random_sample()
        U = chance
        for particle in self.particle_cloud:
            # particle.w /= self.normalizer
            j = -1
            s = 0
            while(s <= U) and (j < self.n_particles):
                j += 1
                s += particle.w
            if j < self.n_particles:
                newParticleCloud.append(deepcopy(self.particle_cloud[j]))
                U += Minv
            else:
                newParticleCloud.append(deepcopy(self.particle_cloud[j-1]))

        self.particle_cloud = newParticleCloud
        self.publish_particles()

    def move_particles(self, control_action):
        for particle in self.particle_cloud:
            particle.x += control_action[0] + normal(0, self.sigma_motion_model)
            particle.y += control_action[1] + normal(0, self.sigma_motion_model)
            particle.theta = ((particle.theta + control_action[2]) + normal(0, self.sigma_motion_model)) % 360
        self.publish_particles()

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            Computed by taking the weighted average of poses.
        """
        # first make sure that the particle weights are normalized
        x = 0
        y = 0
        theta = 0
        angles = []
        for particle in self.particle_cloud:
            x += particle.x
            y += particle.y
            v = [math.cos(math.radians(particle.theta)),
                 math.sin(math.radians(particle.theta))]
            angles.append(v)

        tot_vector = np.sum(angles, axis=0)
        # sum vectors
        angle = math.atan2(tot_vector[1], tot_vector[0])
        # comes in radians for -pi to pi
        x /= self.n_particles
        y /= self.n_particles
        theta = math.degrees(angle) + 180

        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.robot_pose = Pose(position=Point(x=x, y=y),
                               orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1],
                                                      z=orientation_tuple[2], w=orientation_tuple[3]))

        # print(self.robot_pose)

        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = self.odom_frame
        pose = PoseStamped()
        pose.pose = self.robot_pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    try:
        pf = ParticleFilter()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass
