#!/usr/bin/env python3
from __future__ import print_function
import rospy
from ros_unreal_interface.srv import *
from ros_unreal_interface.msg import *

connect_service = rospy.ServiceProxy('disconnect_unreal', DisconnectService)


def start_node():
	global connect_service
	rospy.init_node("ciao", anonymous=True)
	message = -1
	resp = connect_service(message)
	print(resp)


if __name__ == '__main__':
	start_node()
