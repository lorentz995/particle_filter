from .sensor import Sensor
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import sys
import traceback


class Lidar(Sensor):
    def __init__(self, unreal_settings, obs_settings, player_topic, scan_frame='base_link', scan_cloud_topic='/scan_cloud', publish=True, queue_size=1):
        super().__init__()
        self.__METERS_TO_UNREAL_UNIT = unreal_settings['METERS_TO_UNREAL_UNIT']

        self.__scan_frame = scan_frame
        self.__range_max = obs_settings['lidar_range']
        self.__fields = get_fields()

        if publish:
            self.__publisher = rospy.Publisher('{}{}'.format(player_topic, scan_cloud_topic), PointCloud2, queue_size=queue_size)
        else:
            self.__publisher = None

    def change_settings(self):
        pass

    def publish_observation(self, data):
        # Mirror y axis
        #data[:, :, 1] = data[:, :, 1] * -1
        intensity = np.sqrt(data[:, :, 0]**2 + data[:, :, 1]**2 + data[:, :, 2]**2)
        intensity = np.expand_dims(intensity, axis=2)
        data = np.concatenate((data, intensity), axis=2)
        data = data / self.__METERS_TO_UNREAL_UNIT
        if self.__publisher is not None:
            pc_message = self.build_pc_from_data(data)
            self.__publisher.publish(pc_message)

    def build_pc_from_data(self, data):
        h = Header()
        h.frame_id = self.__scan_frame
        # h.stamp = rospy.Time.now()

        pc_message = PointCloud2()
        pc_message.header = h
        pc_message.height = np.array(1, dtype=np.uint32)
        pc_message.width = np.array(np.prod(data.shape[:-1]), dtype=np.uint32)
        # print('width: ', pc_message.width)
        pc_message.fields = self.__fields
        pc_message.is_bigendian = False
        pc_message.point_step = np.array(16, dtype=np.uint32)
        pc_message.row_step = np.array(pc_message.point_step * pc_message.width, dtype=np.uint32)
        # print('row_step: ', pc_message.row_step)
        data_list = list(data.flatten())
        # print('data flatten: ', len(data_list))
        buf = struct.pack('%sf' % len(data_list), *data_list)
        # print('len data: ', len(list(buf)))
        pc_message.data = buf
        pc_message.is_dense = False
        return pc_message


def get_fields():
    x_field = PointField()
    x_field.name = "x"
    x_field.offset = np.array(0, dtype=np.uint32)
    x_field.datatype = np.array(7, dtype=np.uint8)
    x_field.count = np.array(1, dtype=np.uint32)

    y_field = PointField()
    y_field.name = "y"
    y_field.offset = np.array(4, dtype=np.uint32)
    y_field.datatype = np.array(7, dtype=np.uint8)
    y_field.count = np.array(1, dtype=np.uint32)

    z_field = PointField()
    z_field.name = "z"
    z_field.offset = np.array(8, dtype=np.uint32)
    z_field.datatype = np.array(7, dtype=np.uint8)
    z_field.count = np.array(1, dtype=np.uint32)

    intensity_field = PointField()
    intensity_field.name = "intensity"
    intensity_field.offset = np.array(8, dtype=np.uint32)
    intensity_field.datatype = np.array(7, dtype=np.uint8)
    intensity_field.count = np.array(1, dtype=np.uint32)

    return [x_field, y_field, z_field, intensity_field]
