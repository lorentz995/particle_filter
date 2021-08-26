import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sklearn.neighbors import NearestNeighbors
import rosbag


class Map:
    def __init__(self):
        # Loading the map from rosbag
        map = rosbag.Bag('apples.bag')
        for topic, msg, t in map.read_messages(topics=['map']):
            self.map = msg
        map.close()
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=100, latch=True)
        self.map_pub.publish(self.map)


        """ Likelihood Field model """
        # The coordinates of each grid cell in the map
        X = np.zeros((self.map.info.width * self.map.info.height, 2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1
        # use super fast scikit learn nearest neighbor algorithm

        nbrs = NearestNeighbors(n_neighbors=1,
                                algorithm="ball_tree").fit(occupied)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = np.zeros((self.map.info.width, self.map.info.height))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                self.closest_occ[i, j] = \
                    distances[curr][0] * self.map.info.resolution
                curr += 1
        self.occupied = occupied
        print("Map loaded successfully!")

    def get_map_info(self):
        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        # array_255 = np.where(array_255 == -1, 0, array_255)
        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        # array255 = np.rot90(array_255) # uncomment if you want to rotate 90 degrees clockwise

        return self.permissible_region, self.map.info

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = (x - self.map.info.origin.position.x)/self.map.info.resolution
        y_coord = (y - self.map.info.origin.position.y)/self.map.info.resolution
        # x_coord = x / self.map.info.resolution
        # y_coord = y / self.map.info.resolution
        if type(x) is np.ndarray:
            x_coord = x_coord.astype(np.int)
            y_coord = y_coord.astype(np.int)
        else:
            x_coord = int(x_coord)
            y_coord = int(y_coord)

        is_valid = (x_coord >= 0) & (y_coord >= 0) & (x_coord < self.map.info.width) & (y_coord < self.map.info.height)
        if type(x) is np.ndarray:
            distances = np.float('nan') * np.ones(x_coord.shape)
            distances[is_valid] = self.closest_occ[x_coord[is_valid], y_coord[is_valid]]
            return distances
        else:
            return self.closest_occ[x_coord, y_coord] if is_valid else float('nan')


if __name__ == '__main__':
    rospy.init_node('test_map')
    m = Map()
    rospy.spin()
