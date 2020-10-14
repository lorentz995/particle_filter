import rospy, sys, cv2, time
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import multiprocessing as mp
import threading

class ROSEnv():
    def __init__(self, id, environment, observations_list_step, ros_sensor_set, action_manager, player_topic, unreal_settings):
        self.__id = id
        self.__env = environment
        self.__observations_list_step = observations_list_step
        self.__sensor_set = ros_sensor_set
        self.__action_manager = action_manager
        self.__player_topic = player_topic
        self.__unreal_settings = unreal_settings
        self.__running = True

        self.__last_cmd_vel = Twist()
        self.__last_cmd_vel.linear.x = 0
        self.__last_cmd_vel.linear.y = 0
        self.__last_cmd_vel.linear.z = 0
        self.__last_cmd_vel.angular.x = 0
        self.__last_cmd_vel.angular.y = 0
        self.__last_cmd_vel.angular.z = 0

        thread1 = threading.Thread(target=self.spin)
        thread2 = threading.Thread(target=self.update_cmd_vel)
        thread1.start()
        thread2.start()


    def spin(self):
        # FPS
        actions_on_second = self.__unreal_settings["MAX_FPS"]  # ATTENZIONE ALLA STABILITA': SE SI METTE UN NUMERO TROPPO ALTO UNREAL DIVENTA INSTABILE

        self.__last_time = time.time()
        while self.__running:
            current_time = time.time()
            actions = self.__action_manager.get_actions(self.__last_cmd_vel)
            _, observations = self.__env.env_step(actions, self.__observations_list_step)
            for obs_name, obs in zip(self.__observations_list_step, observations):
                self.__sensor_set[obs_name].publish_observation(obs)

            # Check FPS
            # print("FPS: ", 1/(current_time - self.__last_time))
            self.__last_time = current_time

            time.sleep(1/actions_on_second)
        self.__env.close_connection()


    def quit_env(self):
        self.__running = False


    def update_cmd_vel(self):
        cmd_vel_topic = '{}{}/cmd_vel'.format(self.__player_topic, self.__id)
        while self.__running:
            try:
                self.__last_cmd_vel = rospy.wait_for_message(cmd_vel_topic, Twist)
            except:
                pass

