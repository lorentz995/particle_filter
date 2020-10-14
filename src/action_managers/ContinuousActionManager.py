from .action_manager import ActionManager
from geometry_msgs.msg import Twist
import numpy as np
import time
from collections import deque


class ContinuousActionManager(ActionManager):
    def __init__(self, unreal_settings):
        super().__init__()
        self.__METERS_TO_UNREAL_UNIT = unreal_settings['METERS_TO_UNREAL_UNIT']
        self.__MAX_FPS = unreal_settings['MAX_FPS']

        # Autodetect FPS
        self.__last_time = time.time()
        self.__fps_queue = deque(maxlen=100)
        self.__fps_queue.append(self.__MAX_FPS)

    def change_settings(self):
        pass

    def get_actions(self, data):
        current_time = time.time()
        current_fps = 1 / (current_time - self.__last_time)
        if current_fps <= self.__MAX_FPS:
            self.__fps_queue.append(current_fps)
        fps_mean = sum(self.__fps_queue) / len(self.__fps_queue)
        # print("FPS: ", fps_mean)
        self.__last_time = current_time
        vel_x = data.linear.x
        angular_vel = -data.angular.z

        # Convert to unreal
        forward = (vel_x / fps_mean) * self.__METERS_TO_UNREAL_UNIT
        turn = np.rad2deg(angular_vel / fps_mean)
        actions = [turn, forward]
        return actions
