#!/usr/bin/env python3

import rospy
import sys


from nav_msgs.msg import *
from std_msgs.msg import *


def map_stuff():
    global map_pub
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    rospy.init_node('turtlebot_map', anonymous=True)
    while not rospy.is_shutdown():
        create_map()
        rospy.spin()

def create_map( ):
    test_map = OccupancyGrid()
    test_map.info.resolution = 0.05
    test_map.info.width = 201
    test_map.info.height = 265
    test_map.info.origin.position.x = -0.65
    test_map.info.origin.position.y = -0.35
    test_map.info.origin.position.z = 0.0
    test_map.info.origin.orientation.x = 0.0
    test_map.info.origin.orientation.y = 0.0
    test_map.info.origin.orientation.z = 0.0
    test_map.info.origin.orientation.w = 0.0
    test_map.data = []
    # for i in range(0, 100):
        # test_map.data.append(i)
    # print(test_map)
    # test_map.data =
    # map_pub = rospy.Publisher('/map', OccupancyGrid)
    map_pub.publish(test_map)


if __name__ == '__main__':
    try:
        map_stuff()
    except rospy.ROSInterruptException:
        pass