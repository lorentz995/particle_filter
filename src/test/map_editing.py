import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time
import rosbag





def get_map(map_msg):
    # TODO: ruotare la mappa di 90°, ripubblicandola su /occupancygrid e per farlo,
    # invertire la y (cambio di segno)
    # 0: permissible, -1: unmapped, 100: blocked
    array_255 = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
    # Writing into bag
    grid = OccupancyGrid()
    grid = map_msg
    grid.header.frame_id = 'odom'  # TODO: Forse c'è una trasformazione tra odom e map

    grid.data = np.asarray(array_255).flatten()

    try:
        bag = rosbag.Bag('apples.bag', 'w')
        bag.write('map', grid)
    finally:
        bag.close()
        print("finish")



if __name__ == '__main__':
    try:
        rospy.init_node('Map_rotate')
        map_sub = rospy.Subscriber("projected_map", OccupancyGrid, get_map)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('Main Error')
        pass