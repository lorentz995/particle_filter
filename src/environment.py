#!/usr/bin/env python3
from __future__ import print_function
import rospy, sys, cv2, time, os
import numpy as np
from geometry_msgs.msg import Twist
from ros_unreal_interface.srv import *
from ros_unreal_interface.msg import *
from unreal_api.environment import Environment
from ROSEnv import ROSEnv
import threading
import queue
import json
import importlib
import traceback

# Global Variables
envs = {}
id_free = 0
player_topic = "/player"

# Settings Path
pathname = os.path.dirname(sys.argv[0])
abs_path = os.path.abspath(pathname)
settings_path = abs_path+"/settings/"


def build_environment(port, out_queue, render, address, sensor_settings, action_manager_settings, reset_manager_settings=None, observations_step=None, observations_reset=None):
    env = Environment(port, address=address, sensor_settings=sensor_settings, action_manager_settings=action_manager_settings, reset_manager_settings=reset_manager_settings, render=render, observations_step=observations_step, observations_reset=observations_reset)
    out_queue.put(env)


def ord_envs(env):
    return env.port


def start_node():
    rospy.init_node('unreal_interface')
    connect_service = rospy.Service('connect_unreal', ConnectService, connectFunction)
    disconnect_service = rospy.Service('disconnect_unreal', DisconnectService, disconnectFunction)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


def connectFunction(req):
    global envs, id_free, player_topic, settings_path
    message = req.message
    result = False
    json_path_unreal = str(settings_path + 'unreal_settings.json')

    try:
        envs_list = []
        sensor_list = []
        action_manager_list = []

        f_unreal = open(json_path_unreal)
        unreal_settings = json.load(f_unreal)

        ports = list(message.ports)
        json_settings = list(message.json_settings)
        render = message.render
        address = message.address

        out_queue = queue.Queue()
        threads = []
        for port, json_setting in zip(ports, json_settings):
            settings = json.loads(json_setting)
            thread = threading.Thread(target=build_environment, args=(port, out_queue, render, address),
                                      kwargs=settings)
            threads.append(thread)
            sensors_used = settings["observations_step"]
            sensor_list.append(sensors_used)

            action_manager_type = list(settings["action_manager_settings"].keys())[0]
            action_manager_list.append(action_manager_type)

        [thread.start() for thread in threads]
        [thread.join() for thread in threads]

        for _ in range(len(ports)):
            envs_list.append(out_queue.get())

        envs_list.sort(key=ord_envs)

        for env, sensors_used, action_manager_type in zip(envs_list, sensor_list, action_manager_list):
            sensor_set = build_player_sensors(id_free, sensors_used, env)
            action_manager = build_player_action_manager(action_manager_type)
            ros_env = ROSEnv(id_free, env, sensors_used, sensor_set, action_manager, player_topic, unreal_settings)
            # ros_env.start()
            envs[id_free] = ros_env
            id_free += 1
        result = True
    except Exception:
        print('Exception')
        traceback.print_exc(file=sys.stdout)

    return ConnectServiceResponse(bool(result))


def build_player_sensors(id, sensors_used, env):
    global settings_path, player_topic
    sensor_set = {}
    for obs in sensors_used:
        obs_settings = env.sensor_set[obs].settings
        module = importlib.import_module('sensors.{}'.format(obs))
        class_ = getattr(module, obs)
        json_path = str(settings_path + obs + '.json')
        f = open(json_path)
        settings = json.load(f)
        settings["player_topic"] = str(player_topic + str(id))
        settings["obs_settings"] = obs_settings
        json_path_unreal = str(settings_path + 'unreal_settings.json')
        f_unreal = open(json_path_unreal)
        settings["unreal_settings"] = json.load(f_unreal)
        instance = class_(**settings)
        sensor_set[obs] = instance
    return sensor_set


def build_player_action_manager(action_manager_type):
    module = importlib.import_module('action_managers.{}'.format(action_manager_type))
    class_ = getattr(module, action_manager_type)
    json_path_unreal = str(settings_path + 'unreal_settings.json')
    f_unreal = open(json_path_unreal)
    settings = dict()
    settings['unreal_settings'] = json.load(f_unreal)
    instance = class_(**settings)
    return instance


def disconnectFunction(req):
    global envs, id_free, player_topic
    result = False
    rospy.loginfo("DisconnectService Requested")
    message = int(req.message)
    try:
        if message < 0:
            keys = list(envs.keys())
            print(keys)
            for key in keys:
                envs[key].quit_env()
            envs = {}
            id_free = 0
            result = True
        else:
            env = envs[message]
            env.quit_env()
            del envs[message]
            result = True
    except Exception:
        print('Exception')
        traceback.print_exc(file=sys.stdout)
    return DisconnectServiceResponse(bool(result))


if __name__ == '__main__':
    start_node()

