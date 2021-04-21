#!/usr/bin/env python3
from __future__ import print_function
import rospy
from ros_unreal_interface.srv import *
from ros_unreal_interface.msg import *

connect_service = rospy.ServiceProxy('connect_unreal', ConnectService)


def start_node():
	global connect_service
	rospy.init_node("connect_node", anonymous=True)
	message = ConnectionMessage()
	message.ports = [9734]
	message.json_settings = ['{ "sensor_settings": { "CarEncoder": {}, "RGBCamera": { "width": 240, "height": 240, "channels": "RGB", "FOV" : 90, "show": false }, "Laser": { "start_angle_x": -135.0, "end_angle_x": 135.0, "distance_angle_x": 0.2, "start_angle_y": 0.0, "end_angle_y": 0.0, "distance_angle_y": 1.0, "laser_range": 4000.0, "render": -1 }, "GPS": {}, "Lidar": { "start_angle_x": -45.0, "end_angle_x": 45.0, "distance_angle_x": 0.1, "start_angle_y": 0.0, "end_angle_y": 45.0, "distance_angle_y": 2, "lidar_range": 3000.0 , "render": -1} }, "action_manager_settings": { "ContinuousActionManager": { "command_dict": { "TURN": 0, "STRAIGHT": 1 }, "settings": { } } }, "reset_manager_settings": { "TrackerResetManager": {} }, "observations_step": ["GPS", "RGBCamera", "Lidar"], "observations_reset": ["RGBCamera"] }']
	message.render = True
	message.address = '172.22.192.1'
	resp = connect_service(message)
	print(resp)


if __name__ == '__main__':
	start_node()
