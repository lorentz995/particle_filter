from .sensor import Sensor
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class Laser(Sensor):
    def __init__(self, unreal_settings, obs_settings, player_topic, scan_frame='base_link', scan_topic='/scan', publish=True, queue_size=1):
        super().__init__()
        self.__METERS_TO_UNREAL_UNIT = unreal_settings['METERS_TO_UNREAL_UNIT']
        scan_2d = obs_settings['start_angle_y'] == obs_settings['end_angle_y']
        self.__scan_frame = scan_frame

        if publish and scan_2d:
            self.__publisher = rospy.Publisher('{}{}'.format(player_topic, scan_topic), LaserScan, queue_size=queue_size)
            self.__scan_msg = self.build_laser_scan_msg(obs_settings)
        else:
            self.__publisher = None
            self.__scan_msg = None

    def change_settings(self):
        pass

    def publish_observation(self, data):
        if self.__publisher is not None:
            data = np.flip(data, axis=1)
            data = (1 - data) * self.__scan_msg.range_max
            h = Header()
            h.frame_id = self.__scan_frame
            h.stamp = rospy.Time.now()
            self.__scan_msg.header = h
            self.__scan_msg.ranges = list(data[0])
            self.__scan_msg.intensities = []
            self.__publisher.publish(self.__scan_msg)

    def build_laser_scan_msg(self, obs_settings):
        h = Header()
        h.frame_id = self.__scan_frame
        h.stamp = rospy.Time.now()
        laser_scan_msg = LaserScan()
        laser_scan_msg.header = h
        laser_scan_msg.angle_min = np.deg2rad(obs_settings['start_angle_x'])
        laser_scan_msg.angle_max = np.deg2rad(obs_settings['end_angle_x'])
        laser_scan_msg.angle_increment = np.deg2rad(obs_settings['distance_angle_x'])
        laser_scan_msg.time_increment = 0.0
        laser_scan_msg.range_min = 0.0
        laser_scan_msg.range_max = obs_settings['laser_range'] / self.__METERS_TO_UNREAL_UNIT
        return laser_scan_msg
