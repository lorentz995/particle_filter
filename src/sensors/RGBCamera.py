from .sensor import Sensor
import numpy as np
import math
import cv2
import os
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class RGBCamera(Sensor):
    def __init__(self, unreal_settings, obs_settings, player_topic, camera_frame='camera_link', topic_raw='/image_raw', publish_raw=True, queue_size_raw=1, topic_compressed='/image_raw/compressed', publish_compressed=False, queue_size_compressed=1):
        super().__init__()
        self.__bridge = bridge = CvBridge()
        self.__camera_frame = camera_frame

        self.__publisher_camera_info = rospy.Publisher('{}/camera/camera_info'.format(player_topic), CameraInfo, queue_size=1)

        if publish_raw:
            self.__publisher_raw = rospy.Publisher('{}/camera{}'.format(player_topic, topic_raw), Image, queue_size=queue_size_raw)
        else:
            self.__publisher_raw = None

        if publish_compressed:
            self.__publisher_compressed = rospy.Publisher('{}/camera{}'.format(player_topic, topic_compressed), CompressedImage, queue_size=queue_size_compressed)
        else:
            self.__publisher_compressed = None

        self.__camera_info_msg = self.build_camera_info_msg(obs_settings)



    def change_settings(self):
        pass


    def publish_observation(self, data):
        img_fixed = data * 255
        img_fixed = img_fixed.astype(dtype=np.uint8)

        h = Header()
        h.frame_id = self.__camera_frame
        h.stamp = rospy.Time.now()
        self.__camera_info_msg.header = h
        self.__publisher_camera_info.publish(self.__camera_info_msg)

        if self.__publisher_raw is not None:
            img_raw = self.__bridge.cv2_to_imgmsg(img_fixed, "rgb8")
            img_raw.header.frame_id = self.__camera_frame
            self.__publisher_raw.publish(img_raw)

        if self.__publisher_compressed is not None:
            img_rgb = cv2.cvtColor(img_fixed, cv2.COLOR_BGR2RGB)
            img_compressed = self.__bridge.cv2_to_compressed_imgmsg(img_rgb)
            img_compressed.header.frame_id = self.__camera_frame
            self.__publisher_compressed.publish(img_compressed)

    def build_camera_info_msg(self, obs_settings):
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self.__camera_frame
        camera_info_msg = CameraInfo()
        camera_info_msg.header = h
        camera_info_msg.height = obs_settings['height']
        camera_info_msg.width = obs_settings['width']
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = np.zeros((5), dtype=np.float32)
        camera_info_msg.K = obs_settings['camera_matrix'].flatten()
        camera_info_msg.R = np.array([ 1.,  0.,  0., 0.,  1.,  0., 0.,  0.,  1.])
        camera_info_msg.P = np.concatenate((obs_settings['camera_matrix'], np.zeros((3, 1), dtype=np.float32)), axis=1).flatten()
        camera_info_msg.binning_x = 0
        camera_info_msg.binning_y = 0
        return camera_info_msg