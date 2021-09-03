from ros_numpy import point_cloud2
from ParticleFilter.utils import Particle, printProgressBar
import random
import math
import numpy as np
from numpy.random import random_sample, normal
from copy import deepcopy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion


def initialize_particle_cloud(n_particles, map_info, permissible_region, pose):
    rad = 2  # meters
    particle_cloud = []
    while len(particle_cloud) < n_particles:
        theta = random.random() * (2 * math.pi)

        '''x = random.uniform(pose[0] - rad, pose[0] + rad)
        y = random.uniform(pose[1] - rad, pose[1] + rad)
        '''

        x = random.uniform(map_info.origin.position.x,
                           map_info.width * map_info.resolution + map_info.origin.position.x)
        y = random.uniform(map_info.origin.position.y,
                           map_info.height * map_info.resolution + map_info.origin.position.y)

        x_coord = int((x - map_info.origin.position.x) / map_info.resolution)  # formato mappa
        y_coord = int((y - map_info.origin.position.y) / map_info.resolution)

        try:
            if (0 <= x_coord < map_info.width) and (0 <= y_coord < map_info.height):
                if permissible_region[y_coord][x_coord]:
                    particle = Particle(x, y, theta)
                    particle_cloud.append(particle)
        except IndexError:
            pass

    return particle_cloud


def motion_model(u, old_odom, particle_cloud, sigma_motion_model=0.05):
    for particle in particle_cloud:
        r1 = math.atan2(u[1], u[0]) - old_odom[2]  # orientamento
        d = math.sqrt((u[0] ** 2) + (u[1] ** 2))  # spostamento
        r2 = u[2] - r1

        particle.x += d * math.cos(particle.theta + r1) + normal(0, sigma_motion_model)
        particle.y += d * math.sin(particle.theta + r1) + normal(0, sigma_motion_model)
        particle.theta += (r1 + r2) + normal(0, sigma_motion_model)
    return particle_cloud


def measurement_model(lidar_msg, particle_cloud, map, sigma_likelihood=0.01):
    items, i = len(particle_cloud), 0
    # Initial call to print 0% progress
    printProgressBar(0, items, prefix='Processing new scan: ', suffix='', length=2)

    xyz_cloud = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=lidar_msg)
    if len(xyz_cloud) > 0:
        scans = np.stack(xyz_cloud, axis=0)  # scans is the vector of all received measurement [x,y,z]
        for particle in particle_cloud:
            printProgressBar(i + 1, items, prefix='Processing new scan:', suffix='', length=2)
            i += 1
            x, y, z = scans[:, 0], scans[:, 1], scans[:, 2]
            distances = np.sqrt(x ** 2 + y ** 2 + z ** 2)
            # distances = np.sqrt(x ** 2 + y ** 2)
            angles = np.arctan2(y, x)  # gradi?

            x_part = particle.x + distances * np.cos(particle.theta + angles)
            y_part = particle.y + distances * np.sin(particle.theta + angles)

            d = map.get_closest_obstacle_distance(x_part, y_part)
            wk = np.exp((-d ** 2) / (2 * sigma_likelihood ** 2))
            wk = np.nan_to_num(wk)
            particle.w = np.mean(wk)

            permissible_area, map_info = map.get_map_info()
            x_coord = int((particle.x - map_info.origin.position.x) / map_info.resolution)  # formato mappa
            y_coord = int((particle.y - map_info.origin.position.y) / map_info.resolution)
            # Check if particle is out of map bound
            if not (0 <= x_coord < map_info.width) or not (0 <= y_coord < map_info.height):
                particle.w = 0.0
            elif not permissible_area[y_coord][x_coord]:  # check if particle is on landmark
                particle.w = 0.0
    return particle_cloud


def resample_particles(particle_cloud):
    '''newParticles = [] metodo prof
    max_particles = 0
    for i in range(len(particle_cloud)):
        chance = random_sample()  # * self.normalizer
        j = -1
        s = 0.0
        while (s < chance) and (j < len(particle_cloud) - 1):
            j += 1
            s += particle_cloud[j].w
        newParticles.append(deepcopy(particle_cloud[j]))
    return newParticles'''

    newParticles = []  # metodo fast
    for i in range(len(particle_cloud)):
        # resample the same # of particles
        choice = random_sample()
        # all the particle weights sum to 1
        csum = 0  # cumulative sum
        for particle in particle_cloud:
            csum += particle.w
            if csum >= choice:
                # if the random choice fell within the particle's weight
                newParticles.append(deepcopy(particle))
                break
    return newParticles


def estimate_robot_pose(particle_cloud):
    x = 0
    y = 0
    theta = 0

    for particle in particle_cloud:
        x += particle.x * particle.w
        y += particle.y * particle.w
        theta += particle.w * particle.theta

    var_x = sum(particle.w * ((particle.x - x) ** 2) for particle in particle_cloud)
    var_y = sum(particle.w * ((particle.y - y) ** 2) for particle in particle_cloud)

    orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, theta)
    robot_pose = Pose(position=Point(x=x, y=y, z=0),
                      orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1],
                                             z=orientation_tuple[2], w=orientation_tuple[3]))

    return robot_pose, var_x, var_y
