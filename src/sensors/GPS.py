from .sensor import Sensor
import numpy as np
import cv2
import rospy
import tf
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from std_msgs.msg import Header


def change_ref_system(x, y, yaw, x0, y0, yaw0, thr=1e-3):
    yaw_ = np.radians(0)

    A = np.array([
            [np.cos(yaw_), np.sin(yaw_), 0, -x*np.cos(yaw_) - y*np.sin(yaw_)],
            [-np.sin(yaw_), np.cos(yaw_), 0, x*np.sin(yaw_) - y*np.cos(yaw_)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    [x_, y_, z_, _] = np.dot(A, np.array([x0, y0, 0, 1]))

    R = np.array([
        [np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    [x1, y1, z1] = np.dot(R, np.array([x_, y_, z_]))

    yaw_ = np.fmod((yaw0 - yaw), 2 * np.pi)

    if np.abs(x1) < thr:
        x1 = 0
    if np.abs(y1) < thr:
        y1 = 0

    return x1, y1, yaw_


class GPS(Sensor):
    def __init__(self, unreal_settings, obs_settings, player_topic, base_frame='base_link', odom_frame='odom', odom_topic='/odom', publish_odom=True, queue_size_odom=1, tf_broadcast=True, imu_frame='imu', imu_topic='/imu', publish_imu=True, queue_size_imu=1, path_topic='/path', publish_path=True, queue_size_path=1, distance_thr=0.1):
        super().__init__()
        self.__METERS_TO_UNREAL_UNIT = unreal_settings['METERS_TO_UNREAL_UNIT']
        self.thr = 1e-3

        self.__odom_frame = odom_frame
        self.__base_frame = base_frame
        self.__imu_frame = imu_frame

        self.__current_time, self.__last_time, self.__last_yaw, self.__last_x, self.__last_y = None, None, None, None, None
        self.__last_vx, self.__last_vy, self.__last_vyaw = None, None, None

        self.__tf_broadcaster = tf.TransformBroadcaster()
        self.__tf_broadcast = tf_broadcast

        self.__distance_thr = distance_thr

        if publish_odom:
            self.__odom_publisher = rospy.Publisher('{}{}'.format(player_topic, odom_topic), Odometry, queue_size=queue_size_odom)
            if publish_path:
                self.__path_publisher = rospy.Publisher('{}{}'.format(player_topic, path_topic), Path, queue_size=queue_size_path)
                self.__path = None
        else:
            self.__odom_publisher = None

        # IMU DOESN'T WORK
        if publish_imu:
            self.__imu_publisher = rospy.Publisher('{}{}'.format(player_topic, imu_topic), Imu, queue_size=queue_size_imu)
        else:
            self.__imu_publisher = None

    def change_settings(self):
        pass

    def publish_observation(self, data):
        # One time step delay to compute velocities next step
        if self.__current_time is None:
            self.__current_time = rospy.Time.now()
            self.__last_time = rospy.Time.now()
            self.__last_yaw = np.deg2rad(data[10])
            self.__last_x = data[6] / self.__METERS_TO_UNREAL_UNIT
            self.__last_y = data[7] / self.__METERS_TO_UNREAL_UNIT
            return

        # Two time steps delay to compute accelerations next step
        if self.__last_vx is None:
            self.__current_time = rospy.Time.now()
            current_x = data[6] / self.__METERS_TO_UNREAL_UNIT
            current_y = data[7] / self.__METERS_TO_UNREAL_UNIT
            current_yaw = np.deg2rad(data[10])
            dt = (self.__current_time - self.__last_time).to_sec()

            x, y, yaw = change_ref_system(self.__last_x, self.__last_y, self.__last_yaw, current_x, current_y, current_yaw, self.thr)
            # x, y, yaw = (current_x - self.__last_x), (current_y - self.__last_y), (current_yaw - self.__last_yaw)

            self.__last_vx = x / dt
            self.__last_vy = y / dt
            self.__last_vyaw = yaw / dt

            self.__last_time = self.__current_time
            self.__last_x = current_x
            self.__last_y = current_y
            self.__last_yaw = current_yaw
            return


        self.__current_time = rospy.Time.now()
        current_x = data[6] / self.__METERS_TO_UNREAL_UNIT
        current_y = data[7] / self.__METERS_TO_UNREAL_UNIT
        current_yaw = np.deg2rad(data[10])
        dt = (self.__current_time - self.__last_time).to_sec()
        quat = tf.transformations.quaternion_from_euler(0, 0, current_yaw)

        # Odometry
        if self.__odom_publisher is not None:
            vx = (current_x - self.__last_x) / dt
            vy = (current_y - self.__last_y) / dt
            v_yaw = (current_yaw - self.__last_yaw) / dt

            if self.__tf_broadcast:
                self.__tf_broadcaster.sendTransform(
                    (current_x, current_y, 0.),
                    quat,
                    self.__current_time,
                    self.__base_frame,
                    self.__odom_frame
                )

            if self.__path_publisher is not None:
                curr_ps = PoseStamped()
                curr_ps.header.stamp = self.__current_time
                curr_ps.header.frame_id = self.__odom_frame
                curr_pose = Pose()
                curr_pose.position = Point(current_x, current_y, 0.)
                curr_pose.orientation = Quaternion(*quat)
                curr_ps.pose = curr_pose
                if self.__path is None:
                    self.__path = [curr_ps]
                else:
                    last_pose = self.__path[-1].pose
                    last_position = last_pose.position
                    distance = np.sqrt((current_x - last_position.x)**2 + (current_y - last_position.y)**2)
                    if distance > self.__distance_thr:
                        self.__path.append(curr_ps)
                path = Path()
                path.header.stamp = self.__current_time
                path.header.frame_id = self.__odom_frame
                path.poses = self.__path
                self.__path_publisher.publish(path)

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.__current_time
            odom.header.frame_id = self.__odom_frame
            # set the position
            odom.pose.pose = Pose(Point(current_x, current_y, 0.), Quaternion(*quat))
            # set the covariance of the position
            diag = 0.017
            odom.pose.covariance = [diag,0.0,0.0,0.0,0.0,0.0,
                                    0.0,diag,0.0,0.0,0.0,0.0,
                                    0.0,0.0,diag,0.0,0.0,0.0,
                                    0.0,0.0,0.0,diag,0.0,0.0,
                                    0.0,0.0,0.0,0.0,diag,0.0,
                                    0.0,0.0,0.0,0.0,0.0,diag] 

            # set the velocity
            odom.child_frame_id = self.__base_frame
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, v_yaw))
            # set the covariance of the velocity
            odom.twist.covariance = [diag,0.0,0.0,0.0,0.0,0.0,
                                    0.0,diag,0.0,0.0,0.0,0.0,
                                    0.0,0.0,diag,0.0,0.0,0.0,
                                    0.0,0.0,0.0,diag,0.0,0.0,
                                    0.0,0.0,0.0,0.0,diag,0.0,
                                    0.0,0.0,0.0,0.0,0.0,diag] 

            # publish the message
            # print(odom)
            self.__odom_publisher.publish(odom)

        if self.__imu_publisher is not None:
            x, y, yaw = change_ref_system(self.__last_x, self.__last_y, self.__last_yaw, current_x, current_y, current_yaw, self.thr)
            # x, y, yaw = (current_x - self.__last_x), (current_y - self.__last_y), (current_yaw - self.__last_yaw)

            vx = x / dt
            vy = y / dt
            v_yaw = yaw / dt

            acc_x = (vx - self.__last_vx) / dt
            acc_y = (vy - self.__last_vy) / dt
            imu = Imu()
            imu.header.stamp = self.__current_time
            imu.header.frame_id = self.__imu_frame

            imu.orientation = Quaternion(*quat)
            imu.angular_velocity = Vector3(0, 0, v_yaw)
            imu.linear_acceleration = Vector3(acc_x, acc_y, 0)
            # imu.linear_acceleration_covariance = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0]


            # print(imu)

            self.__imu_publisher.publish(imu)

            self.__last_vx = vx
            self.__last_vy = vy
            self.__last_vyaw = v_yaw

        self.__last_time = self.__current_time
        self.__last_x = current_x
        self.__last_y = current_y
        self.__last_yaw = current_yaw
