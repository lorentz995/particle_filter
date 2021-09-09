#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from ParticleFilter.map_utils import Map
from ParticleFilter.utils import publish_particles, normalize_particles, pose_to_xytheta, bcolors
from ParticleFilter.models import *
from tf import TransformListener
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Twist, PoseStamped, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header


class ParticleFilter:
    def __init__(self):
        print("Particle Filter is starting up...")
        self.initialized = False
        self.base_frame = "base_link"  # the frame of the robot base
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.n_particles = 1000  # the number of particles to use
        self.particle_cloud = []
        self.current_odom = []
        self.tf_listener = TransformListener()

        # Setup publishers and subscribers
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher("weighs", MarkerArray, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped, queue_size=10)
        self.vel_sub = rospy.Subscriber("player0/cmd_vel", Twist, self.velocity)
        self.map = Map()

        self.permissible_area, self.map_info = self.map.get_map_info()

        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose)
        self.lidar_sub = rospy.Subscriber("player0/scan_cloud", PointCloud2, self.lidar_scan)

    def initial_pose(self, pose_msg):
        xy_theta = pose_to_xytheta(pose_msg.pose.pose)

        print("Creating new uniform particle cloud...")
        self.particle_cloud = initialize_particle_cloud(self.n_particles, self.map_info,
                                                        self.permissible_area, xy_theta)
        self.particle_cloud = normalize_particles(self.particle_cloud)
        publish_particles(self.particle_pub, self.marker_pub, self.particle_cloud, self.odom_frame)

        self.initialized = True

    def velocity(self, vel, alpha1=0.1, alpha2=0.05, alpha3=0.05, alpha4=0.1):
        v = vel.linear.x
        w = vel.angular.z
        if v != 0 or w != 0:
            self.sigma_v = alpha1 * v ** 2 + alpha2 * w ** 2
            self.sigma_w = alpha3 * v ** 2 + alpha4 * w ** 2

    def lidar_scan(self, lidar_msg):
        # print("Lidar scan received...")
        if not self.initialized:
            print(bcolors.OKCYAN + "Ready! Please insert initial pose from rviz" + bcolors.ENDC)
            return

        # need to know how to transform the lidar to the base frame
        if not (self.tf_listener.canTransform(self.base_frame, lidar_msg.header.frame_id, lidar_msg.header.stamp)):
            print("Error: Can't transform lidar to the base frame!")
            return
        else:
            # Getting transform from lidar to the base frame
            p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.base_frame), pose=Pose())
            self.lidar_pose = self.tf_listener.transformPose(self.base_frame, p)

        # need to know how to transform between base and odometric frames
        p = PoseStamped(header=Header(stamp=lidar_msg.header.stamp,
                                      frame_id=self.base_frame), pose=Pose())

        if (rospy.Time.now() - lidar_msg.header.stamp).to_sec() >= 0.1:
            # discard transformations that are too late
            return

        try:
            self.tf_listener.waitForTransform(source_frame=p.header.frame_id,
                                              target_frame=self.odom_frame,
                                              time=lidar_msg.header.stamp,
                                              timeout=rospy.Duration(1))
            self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        except:
            print(bcolors.FAIL + "Error during last transformation!" + bcolors.ENDC)

        new_odom = pose_to_xytheta(self.odom_pose.pose)
        if not self.particle_cloud:
            print("No particle cloud!")
        elif self.current_odom:
            self.old_odom = self.current_odom
            control_action = (new_odom[0] - self.current_odom[0],  # x movement
                              new_odom[1] - self.current_odom[1],  # y movement
                              new_odom[2] - self.current_odom[2])  # yaw movement

            self.current_odom = new_odom

            # Updating only if there is a movement.
            if (abs(control_action[0]) > 0.001 or
                    abs(control_action[1]) > 0.001 or
                    abs(control_action[2]) > 0.001):
                self.particle_cloud = motion_model(u=control_action, old_odom=self.old_odom,
                                                   particle_cloud=self.particle_cloud,
                                                   sigma_v_motion_model=self.sigma_v,
                                                   sigma_w_motion_model=self.sigma_w)

                self.particle_cloud = measurement_model(lidar_msg=lidar_msg,
                                                        particle_cloud=self.particle_cloud,
                                                        map=self.map)

                avg = sum(particle.w for particle in self.particle_cloud) / len(self.particle_cloud)
                self.particle_cloud = normalize_particles(self.particle_cloud)

                self.particle_cloud = resample_particles(self.particle_cloud, avg, self.odom_pose)

                publish_particles(self.particle_pub, self.marker_pub, self.particle_cloud, self.odom_frame)

                # Robot trajectory
                robot_pose = PoseWithCovarianceStamped()
                self.particle_cloud = normalize_particles(self.particle_cloud)
                robot_pose.pose, var_x, var_y = estimate_robot_pose(self.particle_cloud)

                # Creating Pose msg only if covariance of estimated pose is less of certain threshold
                if math.sqrt(var_x ** 2 + var_y ** 2) < 0.5:
                    print(bcolors.HEADER + "New estimated robot pose" + bcolors.ENDC)
                    odom = PoseWithCovarianceStamped()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = self.odom_frame
                    # set the position

                    odom.pose.pose = robot_pose.pose
                    # set the covariance of the position

                    odom.pose.covariance = [var_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, var_y, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    self.pose_pub.publish(odom)
                else:
                    print(bcolors.WARNING + "Uncertainty too high to estimate a new pose of the robot" + bcolors.ENDC)

        else:
            self.current_odom = new_odom
            return


if __name__ == '__main__':
    rospy.init_node('Particle_Filter')
    pf = ParticleFilter()
    rospy.spin()
