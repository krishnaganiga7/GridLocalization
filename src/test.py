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



#print(maptop(12.02,7.98))

if __name__ == '__main__':
    print("yayyyyy")

    rospy.init_node('pa3',anonymous=True)
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    ho=Twist()
    rospy.Rate(1)
    rospy.sleep(2)
    bag = rosbag.Bag('grid.bag')
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        if(topic == 'Observations'):
            print msg
        print msg
    bag.close()


    ##print(ls)
