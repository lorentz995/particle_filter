import numpy as np
import cv2
import rospy
import tf
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from std_msgs.msg import Header
from tf import TransformListener, TransformBroadcaster
import time

rospy.init_node('test')
tf_listener = TransformListener()
time.sleep(1)


p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='base_link'), pose=Pose())
tf_waiting = tf_listener.waitForTransform(source_frame=p.header.frame_id,
                                                           target_frame='odom',
                                                           time=rospy.Time.now(),
                                                           timeout=rospy.Duration(1))


odom_pose = tf_listener.transformPose('odom', p)
print(odom_pose)