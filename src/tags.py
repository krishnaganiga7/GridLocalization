#! /usr/bin/env python
import time
from visualization_msgs.msg import Marker
import ros
import rospy
from markers_example import *
list=((1.25,5.25),(1.25,3.25),(1.25,1.25),(4.25,1.25),(4.25,3.25),(4.25,5.25))
rospy.init_node("trunks")
rviz_pub=rospy.Publisher("rviz",Marker,queue_size=1)
while(1):

    display_cube_list(list,rviz_pub)
    time.sleep(2)
