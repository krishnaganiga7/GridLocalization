#!/usr/bin/env python
import rospy
import numpy as np
from os import sys
from math import atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import rosbag
from scipy.stats import norm
from markers_example import *
from visualization_msgs.msg import Marker



def d2c(x,y,t):
    global ds
    x=0.2*x+0.1
    y=0.2*y+0.1
    t=ds*t+ds/2
    return (x,y,t)

def c2d(x,y,t):
    global ds
    x=int(x/0.2)
    y=int(y/0.2)
    t=int(t/ds)

    return (x,y,t)

def normpdf(x, mean, sd):
    #print("Normm",x,mean,sd)
    var = float(sd)**2
    denom = (2*math.pi*var)**0.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom






if __name__ == '__main__':
    ds=90
    print("yayyyyy")
    dim=int(360/ds)
    bel=np.zeros((35,35,dim))
    bel[11,27,2]=0.8
    belbar=np.zeros((35,35,dim))
    listr=[]
    list1=[]
    rospy.init_node("Kakarot")
    rviz_pub=rospy.Publisher("rviz",Marker,queue_size=1)
    list=((1.25,5.25),(1.25,3.25),(1.25,1.25),(4.25,1.25),(4.25,3.25),(4.25,5.25))



    bag = rosbag.Bag(rospy.myargv(argv=sys.argv)[1])
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        if(topic == 'Movements'):
            u=msg
            print(u.timeTag)
            max=0
            delrot1=np.degrees((euler_from_quaternion([u.rotation1.x,u.rotation1.y,u.rotation1.z,u.rotation1.w])))[2]
            if(delrot1<0):
                delrot1+=360
            deltrans=u.translation
            delrot2=np.degrees((euler_from_quaternion([u.rotation2.x,u.rotation2.y,u.rotation2.z,u.rotation2.w])))[2]
            if(delrot2<0):
                delrot2+=360
            eta1=0
            for dp in range(35):
                for dq in range(35):
                    for dr in range(4):
                        # if(bel[dp,dq,dr]<0.00001):
                        #    continue
                        # print(dp,dq,dr)
                        for di in range(35):
                            for dj in range(35):
                                for dt in range(4):
                                    #cello=(p,q,r)
                                    #cellit=(i,j,t)
                                    #print(p,q,r)
                                    (p,q,r)=d2c(dp,dq,dr)
                                    (i,j,t)=d2c(di,dj,dt)
                                    #check if reparint the smae names will cause conflitc IF ERROR
                                    ama=np.degrees(atan2((q-j),(p-i)))
                                    if(ama<0):
                                        ama=360+ama



                                    delhyprot1= ama-t
                                    #print("PQR is",p,q,r)
                                    if(delhyprot1>360):
                                        delhyprot1=delhyprot1%360
                                    # if(delhyprot1>720):
                                    #     delhyprot1-720
                                    if(delhyprot1<0):
                                        modd=np.abs(delhyprot1)%360
                                        delhyprot1=360-modd
                                    # if(delhyprot1<720):
                                    #     delhyprot1+720
                                    delhyptrans=math.sqrt( (p-i)**2 + (q-j)**2 )
                                    delhyprot2=r-t-delhyprot1
                                    if(delhyprot2>360):
                                        delhyprot2=delhyprot2%360
                                    if(delhyprot2<0):
                                        modd2=np.abs(delhyprot2)%360
                                        delhyprot2=360-modd2
                                    #print("the hyp del is",delhyprot1)
                                    #print("the hyp del rot is",delrot1)
                                    #check delhyprot1 and 2 for angle mess up
                                    p1=normpdf(delhyprot1-delrot1,0,45)
                                    #this 45 degree is  for a partcular dsize. CHANGE IT IF CHANGE DS
                                    p2=normpdf(deltrans-delhyptrans,0,10)
                                    p3=normpdf(delhyprot2-delrot2,0,45)
                                    #print("the values of p1 areee",p1,p2,p3)
                                    belbar[dp,dq,dr]=belbar[dp,dq,dr]+(bel[di,dj,dt]*p1*p2*p3)
                        eta1=eta1+belbar[dp,dq,dr]

                        if(belbar[dp,dq,dr]>max):
                            max=belbar[dp,dq,dr]
                            (o,m,n)=(dp,dq,dr)

            
            belbar=belbar/eta1




        if(topic=='Observations'):
            v=msg
            bear=np.degrees((euler_from_quaternion([v.bearing.x,v.bearing.y,v.bearing.z,v.bearing.w])))[2]
            r=v.range
            inde=v.tagNum
            maxi=0
            (x,y)=list[inde]
            eta=0
            for i in range(35):
                for j in range(35):
                    for t in range(4):
                        (a,b,c)=d2c(i,j,t)
                        asa=np.degrees(atan2((y-b),(x-a)))
                        if(asa<0):
                            360+asa


                        bearhyp=asa-c
                        rhyp=math.sqrt( (x-a)**2 + (y-b)**2 )
                        pb=norm(0,45).pdf(bear-bearhyp)
                        pra=norm(0,10).pdf(r-rhyp)
                        bel[i,j,t]=pb*pra*belbar[i,j,t]
                        if(bel[i,j,t]>maxi):
                            maxi=bel[i,j,t]
                            (o,m,n)=(i,j,t)
                        eta=eta+bel[i,j,t]
            bel=bel/eta
            #hihh=np.argmax(bel)
            #print("The HIHH is",o,m,n)
            hihhc=d2c(o,m,n)
            mann=1
            print("The belief after ",mann," update is",o,m)

            list1.append((hihhc[0],hihhc[1]))
            list1.append((hihhc[0],hihhc[1]))
            display_line_list(list1,rviz_pub)

            #print(o,m,n)




        #print msg
    bag.close()
    print(belbar)




    ##print(ls)
