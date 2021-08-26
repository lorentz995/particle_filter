#!/usr/bin/env python3
from ParticleFilter.utils import *
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Twist, Vector3, PoseStamped
from std_msgs.msg import Header
from ros_numpy import point_cloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from numpy import array
from numpy.random import random_sample, normal
import random
import cv2
import matplotlib.pyplot as plt
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import KMeans, DBSCAN

def min_get_map(msg):
    global resolution
    resolution = msg.info.resolution
def get_map(msg):
    global resolution
    resolution = msg.info.resolution
    array_255 = np.array(msg.data).reshape(msg.info.height, msg.info.width)


    w, h = msg.info.width, msg.info.height
    data = np.zeros((w, h, 3), dtype=np.uint8)

    for index, i in enumerate(array_255):
        for jndex, j in enumerate(i):

            if (j == -1) or (j == 0):

                data[jndex, index] = [255, 255, 255]
            elif j == 100:

                data[jndex, index] = [0, 0, 0]
    fig = plt.figure(1)
    img = Image.fromarray(data, 'RGB')
    img.save('my.png')
    img = cv2.imread('my.png')
    alberi = Image.open('no-staccionata.png')
    alberi = array(alberi)

    alberi = cv2.cvtColor(alberi, cv2.COLOR_RGB2GRAY)

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 200, 250, apertureSize=5)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, 50, 80)

    landmarks = np.zeros_like(alberi, dtype=bool)
    landmarks[alberi == 0] = 1
    indexes = np.where(landmarks == True)

    land_coord = np.stack((indexes[0], indexes[1]), axis=1)
    #print(land_coord)
    kmeans = KMeans(n_clusters=62) # numero de alberi
    kmeans.fit(land_coord)
    y_kmeans = kmeans.predict(land_coord)
    plt.scatter(land_coord[:, 0], land_coord[:, 1], c=y_kmeans, s=50, cmap='viridis')
    centers = kmeans.cluster_centers_
    plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    #print(y_kmeans, centers)

    plt.show()

    for line in lines:
        x1, y1, x2, y2 = line[0]
        print(x1,y1,x2,y2)
        cv2.line(img,(x1,y1),(x2,y2),(255,0,0))

    # cv2.imwrite('houghlines3.jpg', img)
    fig2 = plt.figure(2)
    plt.imshow(img)
    plt.show()




def lidar_scan(msg):
    global resolution
    xyz_cloud = point_cloud2.pointcloud2_to_xyz_array(cloud_msg=msg)
    scans = []
    for index, scan in enumerate (xyz_cloud):
        scans.append(scan)
    X = np.stack(scans, axis=0)
    X = np.delete(X,2,axis=1)
    cluster_lidar = KMeans(n_clusters=5) # numero de alberi
    cluster_lidar.fit(X)
    clustered = cluster_lidar.predict(X)

    centers = cluster_lidar.cluster_centers_
    print(centers)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node("stoporcodedio")
    lidar_sub = rospy.Subscriber("player0/scan_cloud", PointCloud2, lidar_scan)
    map_sub = rospy.Subscriber("projected_map", OccupancyGrid, min_get_map)
    rospy.spin()