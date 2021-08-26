#!/usr/bin/env python3
from visualization_msgs.msg import Marker, MarkerArray

from ParticleFilter.utils import *
# from test.map_editing import *
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Twist, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from ros_numpy import point_cloud2
import numpy as np
from numpy.random import random_sample, normal
import random
import matplotlib.pyplot as plt


class ParticleFilter:
    def __init__(self):
        self.initialized = False
        rospy.init_node('Particle_Filter')
        # Viz parameters
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = 'map'  # the name of the map frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.path = Path()
        # Parameters
        self.n_particles = 2000  # the number of particles to use
        # self.sigma_measurement = 0.01
        self.sigma_motion_model = 0.01
        self.sigma_likelihood = 0.01
        self.map = None

        self.normalizer = 0
        self.avg = 0
        self.step = 0

        self.particle_cloud = []
        self.current_odom_xy_theta = []
        self.robot_trajectory = []

        # Setup publishers and subscribers
        self.map_sub = rospy.Subscriber("projected_map", OccupancyGrid, self.get_map)
        # self.map_sub = rospy.Subscriber("rotated_map", OccupancyGrid, self.get_map)
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.stoprobot = rospy.Publisher("player0/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("markers", MarkerArray, queue_size=10)

        self.vel_msg = Twist()

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
        print("Map initializing")
        self.map = map_msg
        self.map_resolution = map_msg.info.resolution
        self.map_width = map_msg.info.width
        self.map_heigth = map_msg.info.height
        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1

        '''data = np.zeros((self.map_width, self.map_heigth, 3), dtype=np.uint8)

        for index, i in enumerate(array_255):
            for jndex, j in enumerate(i):

                if (j == -1) or (j == 0):

                    data[jndex, index] = [255, 255, 255]
                elif j == 100:

                    data[jndex, index] = [0, 0, 0]

        img = Image.fromarray(data, 'RGB')
        img.save('my.png')
        img = cv2.imread('my.png')
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        edges = cv2.Canny(gray, 200, 250, apertureSize=5)
        edges = np.stack(edges, axis=1)
        plt.imshow(edges)
        plt.show()'''

        self.map_initializated = True
        self.likelihood = LikelihoodField(map_msg)

        print("mappa inizializzata")

    def initialize_particle_cloud(self, map):
        while len(self.particle_cloud) < self.n_particles:
            x = random.uniform(0, map.info.width * map.info.resolution)

            y = random.uniform(0, map.info.height * map.info.resolution)
            theta = math.radians(random.random() * 360)
            '''x = random.uniform(0, 0.1)
            y = random.uniform(0, 0.1)
            theta = 0'''
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
        print("particelle pubblicate")

        marker_array = []
        for index, particle in enumerate(self.particle_cloud):
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id=self.odom_frame),
                            pose=particle.as_pose(),
                            type=0,
                            # scale=Vector3(x=particle.w * 20, y=particle.w * 10, z=particle.w * 50),
                            scale=Vector3(x=0.01, y=0.01, z=particle.w * 100),
                            id=index,
                            color=ColorRGBA(b=1, a=1))
            marker_array.append(marker)

        self.marker_pub.publish(MarkerArray(markers=marker_array))

    def lidar_scan(self, lidar_msg):
        print("Lidar scan received..")
        if not self.initialized:
            print("Loading...")
            return
        p = PoseStamped(header=Header(frame_id=self.base_frame), pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        new_odom_xy_theta = pose_to_xytheta(self.odom_pose.pose)

        if not self.particle_cloud:
            self.initialize_particle_cloud(self.map)
            print("Generated random particles")
        elif self.current_odom_xy_theta:
            self.old_odom_xy_theta = self.current_odom_xy_theta
            control_action = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                              new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                              new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta

            if (abs(control_action[0]) > 0 or
                    abs(control_action[1]) > 0 or
                    abs(control_action[2]) > 0):
                self.stop()
                self.move_particles(control_action, lidar_msg)

                # self.update_robot_pose()
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

    def measurement_model(self, lidar_msg):

        self.avg = 0
        i = 0
        items = len(self.particle_cloud)
        # Initial call to print 0% progress
        printProgressBar(0, items, prefix='Processing new scan: ', suffix='', length=10)

        xyz_cloud = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=lidar_msg)

        if len(xyz_cloud) > 0:
            scans = np.stack(xyz_cloud, axis=0)
            # Measurement model
            self.normalizer = 0

            scans = np.stack(scans, axis=0)
            '''X = np.stack(scans, axis=0)
            X = np.delete(X, 2, axis=1)
            cluster_lidar = KMeans(n_clusters=10)  # estimate subsampled clusters of PC2
            cluster_lidar.fit(X)
            centers = cluster_lidar.cluster_centers_'''

            for particle in self.particle_cloud:
                printProgressBar(i + 1, items, prefix='Processing new scan:', suffix='', length=10)
                i += 1
                # TODO: usare anche la z
                x1, y1 = scans[:, 0], scans[:, 1]
                distances = np.sqrt(x1 ** 2, y1 ** 2)
                angles = np.arctan2(y1, x1)

                x_part = particle.x + distances * np.cos(particle.theta + angles)
                y_part = particle.y + distances * np.sin(particle.theta + angles)

                d = self.likelihood.get_closest_obstacle_distance(x_part, y_part)

                # wk = 1 / (np.sqrt(2 * np.pi) * self.sigma_likelihood) * np.exp(
                #    -(d ** 2) / (2 * (self.sigma_likelihood ** 2)))

                wk = np.exp((-d ** 2) / (2 * self.sigma_likelihood ** 2))
                wk = np.nan_to_num(wk)
                w = np.mean(wk)

                particle.w = w
                x_coord = int(particle.x / self.map_resolution)
                y_coord = int(particle.y / self.map_resolution)
                if not (0 <= x_coord < self.map_width) or not (0 <= y_coord < self.map_heigth):
                    particle.w = 0.0
                elif not self.permissible_region[y_coord][x_coord]:
                    particle.w = 0.0
            self.normalize()

            print(self.normalizer)
            self.resample_particles()
            # self.resample_fast()
            self.publish_particles()

    def resample_particles(self):
        # self.plot_weigth()
        newParticles = []
        max_particles = 0
        for i in range(len(self.particle_cloud)):
            if self.normalizer < 0.0 and max_particles < (100 / self.avg):
                x = random.uniform(0, self.map_width * self.map_resolution)
                y = random.uniform(0, self.map_heigth * self.map_resolution)
                theta = random.random() * 360

                particle = Particle(x, y, theta, 1 / self.n_particles)
                newParticles.append(particle)
                max_particles = max_particles + 1

            else:
                chance = random_sample()  # * self.normalizer
                j = -1
                s = 0.0
                while (s < chance) and (j < self.n_particles - 1):
                    j += 1
                    s += self.particle_cloud[j].w
                newParticles.append(deepcopy(self.particle_cloud[j]))

        self.particle_cloud = newParticles
        # print("Kidnapping? Added new: ", max_particles, "particles")

    def resample_fast(self):
        newParticles = []
        for i in range(len(self.particle_cloud)):
            # resample the same # of particles
            choice = random_sample()
            # choice = 0.01  # soglia per il resampling del 50%
            # all the particle weights sum to 1
            csum = 0  # cumulative sum
            for particle in self.particle_cloud:
                csum += particle.w
                if csum >= choice:
                    # if the random choice fell within the particle's weight
                    newParticles.append(deepcopy(particle))
                    break
        self.particle_cloud = newParticles

    def plot_weigth(self):
        self.step += 1
        fig = plt.figure(figsize=(4, 4))
        ax = fig.add_subplot(111, projection='3d')
        x, y, w = [], [], []
        for particle in self.particle_cloud:
            x.append(particle.x)
            y.append(particle.y)
            w.append(particle.w)
        ax.scatter(x, y, w, c='r', marker='o')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('W')

        plt.show()

    def move_particles(self, control_action, lidar_msg):

        delta = control_action

        for particle in self.particle_cloud:
            r1 = math.atan2(delta[1], delta[0]) - self.old_odom_xy_theta[2]  # orientamento
            d = math.sqrt((delta[0] ** 2) + (delta[1] ** 2))  # spostamento
            r2 = delta[2] - r1

            # particle.theta += r1 % 360
            particle.x += d * math.cos(particle.theta + r1) + normal(0, 0.01)
            particle.y += d * math.sin(particle.theta + r1) + normal(0, 0.01)
            particle.theta += (r1 + r2) + normal(0, 0.01)
        self.measurement_model(lidar_msg)

        # self.publish_particles()

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            Computed by taking the weighted average of poses.
        """
        # TODO: prendere la media pesata dei cluster delle particelle
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

################# UTILS ######################

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, \
    Pose, Point, Quaternion
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


'''class Landmarks(object):
    def __init__(self, map):
        self.map = map
        # self.landmarks = np.zeros(self.map.info.height, self.map.info.width)
        array_255 = np.array(map.data).reshape((map.info.height, map.info.width))
        self.landmarks = np.zeros_like(array_255, dtype=bool)
        self.landmarks[array_255 == 100] = 1
        self.indexes = np.where(self.landmarks == True)

        self.x_coord = self.indexes[0]
        self.y_coord = self.indexes[1]


    def get_index_landmark(self):
        return self.clustered_tree'''


def pose_to_xytheta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]


class LikelihoodField(object):
    """ Stores an likelihood field for an input map.  An likelihood field returns
        the distance to the closest obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to
            the closest obstacle
    """

    def __init__(self, map):
        '''self.clustered_tree = [(113.24657534, 183.98630137),
                               (65.36619718, 57.76056338),
                               (41.28358209, 155.7761194),
                               (60.43333333, 247.76666667),
                               (173.14084507, 246.06338028),
                               (137.32857143, 57.95714286),
                               (182.82051282, 137.03846154),
                               (41.15384615, 85.75384615),
                               (75.38255034, 10.81879195),
                               (182.52317881, 185.06622517),
                               (65.33802817, 128.02816901),
                               (137.35211268, 141.74647887),
                               (102.13815789, 231.01973684),
                               (33.36601307, 224.5620915),
                               (12.24183007, 69.4248366),
                               (89.24324324, 170.06756757),
                               (89.27027027, 99.98648649),
                               (106.05769231, 24.07692308),
                               (169.30344828, 101.96551724),
                               (164.53846154, 50.83333333),
                               (137.47761194, 169.73134328),
                               (65.68115942, 183.73913043),
                               (137.3, 85.87142857),
                               (12.1589404, 161.85430464),
                               (41.32352941, 127.77941176),
                               (113.38571429, 141.67142857),
                               (113.14084507, 71.81690141),
                               (65.23287671, 85.93150685),
                               (89.30555556, 128.04166667),
                               (89.43055556, 71.97222222),
                               (41.19117647, 169.77941176),
                               (41.45454545, 71.75757576),
                               (137.31944444, 113.81944444),
                               (65.27142857, 156.04285714),
                               (65.33333333, 113.91666667),
                               (113.31944444, 99.86111111),
                               (113.53030303, 156.1969697),
                               (89.07042254, 58.05633803),
                               (89.27027027, 183.94594595),
                               (113.18421053, 85.82894737),
                               (41.546875, 99.546875),
                               (89.2027027, 156.09459459),
                               (137.23188406, 99.75362319),
                               (137.03125, 183.609375),
                               (89.24657534, 85.93150685),
                               (41.28169014, 57.74647887),
                               (137.38571429, 155.75714286),
                               (65.26027397, 71.91780822),
                               (65.41428571, 142.),
                               (137.09090909, 71.72727273),
                               (89.25675676, 113.97297297),
                               (41.46575342, 183.82191781),
                               (89.125, 141.94444444),
                               (65.27777778, 99.88888889),
                               (113.36619718, 58.01408451),
                               (113.19736842, 127.82894737),
                               (65.38571429, 169.91428571),
                               (41.35294118, 113.73529412),
                               (137.36111111, 127.77777778),
                               (41.49230769, 141.8),
                               (113.26086957, 169.92753623),
                               (42, 152),
                               (196, 152),
                               (49, 25),
                               (212, 25),
                               (43, 144),
                               (43, 26),
                               (213, 123),
                               (213, 26)]
        self.clustered_tree = np.stack(self.clustered_tree, axis=0)
        print(self.clustered_tree)'''
        # grab the map from the map server
        self.map = map

        # The coordinates of each grid cell in the map
        X = np.zeros((self.map.info.width * self.map.info.height, 2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1
        # use super fast scikit learn nearest neighbor algorithm

        nbrs = NearestNeighbors(n_neighbors=1,
                                algorithm="ball_tree").fit(occupied)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = np.zeros((self.map.info.width, self.map.info.height))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                self.closest_occ[i, j] = \
                    distances[curr][0] * self.map.info.resolution
                curr += 1
        self.occupied = occupied

    '''def get_obstacle_bounding_box(self):
        """
        Returns: the upper and lower bounds of x and y such that the resultant
        bounding box contains all of the obstacles in the map.  The format of
        the return value is ((x_lower, x_upper), (y_lower, y_upper))
        """
        lower_bounds = self.occupied.min(axis=0)
        upper_bounds = self.occupied.max(axis=0)
        r = self.map.info.resolution
        return ((lower_bounds[0]*r + self.map.info.origin.position.x,
                 upper_bounds[0]*r + self.map.info.origin.position.x),
                (lower_bounds[1]*r + self.map.info.origin.position.y,
                 upper_bounds[1]*r + self.map.info.origin.position.y))'''

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        # x_coord = (x - self.map.info.origin.position.x)/self.map.info.resolution
        # y_coord = (y - self.map.info.origin.position.y)/self.map.info.resolution
        x_coord = x / self.map.info.resolution
        y_coord = y / self.map.info.resolution
        if type(x) is np.ndarray:
            x_coord = x_coord.astype(np.int)
            y_coord = y_coord.astype(np.int)
        else:
            x_coord = int(x_coord)
            y_coord = int(y_coord)

        is_valid = (x_coord >= 0) & (y_coord >= 0) & (x_coord < self.map.info.width) & (y_coord < self.map.info.height)
        if type(x) is np.ndarray:
            distances = np.float('nan') * np.ones(x_coord.shape)
            distances[is_valid] = self.closest_occ[x_coord[is_valid], y_coord[is_valid]]
            return distances
        else:
            return self.closest_occ[x_coord, y_coord] if is_valid else float('nan')


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
