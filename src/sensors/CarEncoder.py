from .sensor import Sensor
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from utils.BicycleKinematicOdometry import BicycleKinematicOdometry


class CarEncoder(Sensor):
    def __init__(self, unreal_settings, obs_settings, player_topic, base_frame='base_link', odom_frame='odom', odom_topic='/odom', publish_odom=True, queue_size_odom=1, tf_broadcast=True, path_topic='/path', publish_path=True, queue_size_path=1, distance_thr=0.1):
        super().__init__()
        self.__kinematicModel = BicycleKinematicOdometry()
        self.__METERS_TO_UNREAL_UNIT = unreal_settings['METERS_TO_UNREAL_UNIT']

        self.__odom_frame = odom_frame
        self.__base_frame = base_frame

        self.__current_time, self.__last_time = None, None
        self.__last_yaw, self.__last_x, self.__last_y = 0.0, 0.0, 0.0

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

    def change_settings(self):
        pass

    def publish_observation(self, data):
        # One time step delay to compute velocities next step
        steerings = [data[0], data[2],
                     data[4], data[6]]
        rotations = [data[1], data[3],
                     data[5], data[7]]
        wheelRadius = data[8]
        Lr = data[10] / 2
        Lf = data[10] / 2

        if self.__last_time is None:
            self.__last_time = rospy.Time.now()
            self.__kinematicModel.SetStaticData(wheelRadius, Lr, Lf)
            return

        self.__current_time = rospy.Time.now()
        dt = (self.__current_time - self.__last_time).to_sec()
        trajectory = self.__kinematicModel.carTrajector(steerings, rotations, delta_t=dt)
        current_x = trajectory[0] / self.__METERS_TO_UNREAL_UNIT
        current_y = trajectory[1] / self.__METERS_TO_UNREAL_UNIT
        current_yaw = trajectory[2]

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
            # set the velocity
            odom.child_frame_id = self.__base_frame
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, v_yaw))
            # publish the message
            # print(odom)
            self.__odom_publisher.publish(odom)

        self.__last_time = self.__current_time
        self.__last_x = current_x
        self.__last_y = current_y
        self.__last_yaw = current_yaw
