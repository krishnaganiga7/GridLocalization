#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import rosbag
from scipy.stats import norm

def q2deg(u):
    (_,_,t)=euler_from_quaternion([u.x,u.y,u.z,u.w])
    dg=(t*180)/math.pi
    return dg

if __name__ == '__main__':
    bag = rosbag.Bag('grid.bag')
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        if(topic == 'Movements'):
            u=msg
            s=q2deg(u.rotation1)
            print(s)
