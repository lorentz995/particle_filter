from ros_numpy import point_cloud2
from ParticleFilter.utils import Particle, printProgressBar
import random
import math
import numpy as np
from numpy.random import random_sample, normal
from copy import deepcopy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion

global num_iteration


def initialize_particle_cloud(n_particles, map_info, permissible_region, pose):
    global num_iteration
    num_iteration = 0
    rad = 1.5  # meters
    particle_cloud = []
    while len(particle_cloud) < n_particles:
        theta = random.random() * (2 * math.pi)

        x = random.uniform(pose[0] - rad, pose[0] + rad)
        y = random.uniform(pose[1] - rad, pose[1] + rad)

        ''' Uncomment this if you want uniform distribution over the map, comment the previous two lines'''

        '''x = random.uniform(map_info.origin.position.x,
                           map_info.width * map_info.resolution + map_info.origin.position.x)
        y = random.uniform(map_info.origin.position.y,
                           map_info.height * map_info.resolution + map_info.origin.position.y)'''

        x_coord = int((x - map_info.origin.position.x) / map_info.resolution)
        y_coord = int((y - map_info.origin.position.y) / map_info.resolution)

        try:
            if (0 <= x_coord < map_info.width) and (0 <= y_coord < map_info.height):
                if permissible_region[y_coord][x_coord]:
                    particle = Particle(x, y, theta)
                    particle_cloud.append(particle)
        except IndexError:
            pass

    return particle_cloud


def motion_model(u, old_odom, particle_cloud, sigma_v_motion_model, sigma_w_motion_model):
    for particle in particle_cloud:
        r1 = math.atan2(u[1], u[0]) - old_odom[2]
        d = math.sqrt((u[0] ** 2) + (u[1] ** 2))
        r2 = u[2] - r1

        particle.x += d * math.cos(particle.theta + r1) + normal(0, sigma_v_motion_model)
        particle.y += d * math.sin(particle.theta + r1) + normal(0, sigma_v_motion_model)
        particle.theta += (r1 + r2) + normal(0, sigma_w_motion_model)
    return particle_cloud


def measurement_model(lidar_msg, particle_cloud, map, sigma_likelihood=0.01):
    items, i = len(particle_cloud), 0
    # Display a progress bar
    printProgressBar(0, items, prefix='Processing new scan: ', suffix='', length=2)

    xyz_cloud = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=lidar_msg)
    if len(xyz_cloud) > 0:
        scans = np.stack(xyz_cloud, axis=0)  # scans is the vector of all received measurement [x,y,z]
        for particle in particle_cloud:
            printProgressBar(i + 1, items, prefix='Processing new scan:', suffix='', length=2)
            i += 1
            x, y, z = scans[:, 0], scans[:, 1], scans[:, 2]
            distances = np.sqrt(x ** 2 + y ** 2 + z ** 2)
            angles = np.arctan2(y, x)

            x_part = particle.x + distances * np.cos(particle.theta + angles)
            y_part = particle.y + distances * np.sin(particle.theta + angles)

            d = map.get_closest_obstacle_distance(x_part, y_part)
            wk = np.exp((-d ** 2) / (2 * sigma_likelihood ** 2))
            wk = np.nan_to_num(wk)
            particle.w = np.mean(wk)

            permissible_area, map_info = map.get_map_info()
            x_coord = int((particle.x - map_info.origin.position.x) / map_info.resolution)
            y_coord = int((particle.y - map_info.origin.position.y) / map_info.resolution)
            # Check if particle is out of map bound
            if not (0 <= x_coord < map_info.width) or not (0 <= y_coord < map_info.height):
                particle.w = 0.0
            elif not permissible_area[y_coord][x_coord]:  # check if particle is on landmark
                particle.w = 0.0
    return particle_cloud


def resample_particles(particle_cloud, avg, odom_pose, avg_threshold=0.2, max_particles=500):
    global num_iteration
    rad = 1.5
    num_iteration += 1
    num_particles = 0
    newParticles = []
    for i in range(len(particle_cloud)):
        # Kidnapping
        if avg < avg_threshold and num_particles < max_particles and num_iteration % 10 == 0:
            theta = random.random() * (2 * math.pi)
            x = random.uniform(odom_pose.pose.position.x - rad, odom_pose.pose.position.x + rad)
            y = random.uniform(odom_pose.pose.position.y - rad, odom_pose.pose.position.y + rad)

            ''' Uncomment this if you want uniform distribution over the map, comment the previous two lines'''

            '''x = random.uniform(map_info.origin.position.x,
                               map_info.width * map_info.resolution + map_info.origin.position.x)
            y = random.uniform(map_info.origin.position.y,
                               map_info.height * map_info.resolution + map_info.origin.position.y)'''

            particle = Particle(x, y, theta, 1 / 500)
            newParticles.append(particle)
            num_particles += 1
        else:  # End of kidnapping

            choice = random_sample()
            csum = 0
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
