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


class Node():

    def __init__(self,parent=None,position=None):

        self.parent=parent
        self.position=position
        self.g=0
        self.h=0

        self.f=0

    def __eq__(self,other):
        return self.position==other.position

global start
global end

def astar():

    maz = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

    a=np.array(maz)
    maz=np.reshape(a,(20,18))
    vis=maz
    start=(11,0)
    startNode=Node(None,start)

    #xav=rospy.get_param('/goalx')
    #yav=rospy.get_param('/goaly')
    #print("The XAV YAV",xav,yav)
    xyav=(4.5,9)
    #xyav=(xav,yav)

    end=postom(xyav)
    #end=(0,13)
    endNode=Node(None,end)

    openl=[]
    closedl=[]
    openl.append(startNode)

    while(len(openl)!=0):
        mini=float('inf')
        for i in openl:
            #print(i.position,i.f)
            if(i.f<mini):


                mini=i.f
                cur=i
        #print("The length of open list",len(openl))
        #print("The CURRENT NODE is",cur.position)
        #print("This is openl",openl)

        openl.remove(cur)
        closedl.append(cur.position)

        if(cur.position==end):
            i=cur
            #print(i.position)
            global path
            while(i.position!=(11,0)):
                i=i.parent
                vis[i.position]=5
                #print(i.position)
                path.append(i.position)
            #print(vis)
            path = path[::-1]
            path.append(end)
            #print[path]
            return 0

        for i in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            #print ("FUUUU")
            #print("we're in child",i)
            cp=(i[0]+cur.position[0],i[1]+cur.position[1])
            #print("The child is",cp, "and it was a obstacle" ,maz[cp])
            if (cp[0]<0) or (cp[0]>len(maz)-1) or (cp[1]<0) or (cp[1]>len(maz[0])-1) or (cp in closedl) or (maz[cp]==1):
                #print("Bad child")
                continue

            xx=0
            #print("the openl is",openl)
            for n in openl:
                if n.position==cp :
                    #print("the math value",math.sqrt(i[0]**2+i[1]**2))
                    f=cur.g+math.sqrt(i[0]**2+i[1]**2)
                    #print("Theeeeee")
                    if(f<n.g):
                        n.g=f
                        n.parent=cur
                        #print("The g and parent of ",np,"was updated to ",f,cur.parent)
                        xx=1

            if(xx==0):

                if(i[0]==1 and i[1]==1):
                    #print("gooooo")
                    #print("The left ele index ",cp[0]-1,cp[1])
                    #print("The down ele index is",cp[0],cp[1]-1)
                    #le=(cp[0]-1,cp[1])
                    #de=(cp[0],cp[1]-1)
                    #print("The left and down maze obtsacle",maz[le],maz[de])

                    if((maz[(cp[0]-1,cp[1])]==1) and (maz[cp[0],cp[1]-1]==1)):
                        #print("diagonal obstacle")
                        continue
                if(i[0]==-1 and i[1]==-1):
                    #print("Herrr")
                    if((maz[cp[0]+1,cp[1]]==1) and (maz[cp[0],cp[1]+1]==1)):
                        #print("diagonal obstacle")
                        continue
                if(i[0]==-1 and i[1]==1):
                    #print("Yayyy")
                    if((maz[cp[0]+1,cp[1]]==1) and (maz[cp[0],cp[1]-1]==1)):
                        #print("diagonal obstacle")
                        continue

                if(i[0]==1 and i[1]==-1):
                    #print("Yayyy")
                    if((maz[cp[0]-1,cp[1]]==1) and (maz[cp[0],cp[1]+1]==1)):
                        #print("diagonal obstacle")
                        continue


                tl=Node(cur,cp)
                tl.g=cur.g+math.sqrt(i[0]**2+i[1]**2)
                #print("The g of",cp," is",tl.g)
                #print(end[0],cp[0],end[1],cp[1])
                tl.h=math.sqrt((end[0]-cp[0])**2+ (end[1]-cp[1])**2)
                #print("The h of",cp," is",tl.h)
                tl.f=tl.g+tl.h

                #print("The f of",cp,"is",tl.f)

                if(tl not in openl):
                    openl.append(tl)
                #print("The length of openl is",len(openl))

def postom(pos):

    r=int(9.5-pos[1])
    c=int(8.5+pos[0])
    resu=(r,c)
    return resu



#print(postom((4.5,9)))

def maptop(r,c):
    pos=[]
    pos.append(c-8.5)
    pos.append(9-r)
    return pos

def laser(msg):
    global tb
    global ls
    global binsize
    global binwidth
    global x
    global y
    global theta
    global gd
    global path
    global ls
    global tb
    global thetad
    global posi
    global ppath
    ls=msg.ranges[0:360]
    ls = np.reshape(ls, (binsize,-1))
    #print(ls)

    bv=[]
    tb=[]

    for i in range(0,binsize):
        count=0
        for j in range(0,binwidth):
            if(ls[i][j]<1.5):
                ls[i][j]=1
                count=count+1

            else:
                ls[i][j]=0
        bv.append(count)
        if(count>4):
            tb.append(1)
        else:
            tb.append(0)




    #print(ls)
    #print(bv)
    print(tb)
    for i in ppath:

        print("trying to reach",i)

        while math.sqrt(sum([(a - b) ** 2 for a, b in zip(posi, i)])) >1:



            gd=atan2((i[1]-y),(i[0]-x))
            d=gd
            pey=math.pi
            if(d<0):
                gd=(2*pey)-abs(d)
            rospy.sleep(2)
            gdd=(gd*180)/(math.pi)
            #print(gdd)
            k=-1
            psd=thetad
            minco=float('inf')
            for p in tb:
                if(p==0):
                    k=k+1
                    cdrf=((k+0.5)*(180))/binsize
                    #print("cdrf is",cdrf)
                    #print("theta is",thetad)
                    cd=thetad-90+cdrf
                    #print("The candidate direction is", cd, "of indice", k)
                    td=abs(cd-gdd)
                    curd=abs(cd-thetad)
                    pd=abs(cd-psd)
                    cf=td+curd+pd
                    #print("The cost function is",cf)
                    if(cf<minco):
                        minco=cf
                        gold=cd
                        goldr=(gold*math.pi)/180
                        rifle=k






                else:
                    k=k+1
                #rospy.sleep(1)
            print("The chosen direction is",cd, "and index is",rifle)

            if(abs(thetad-gold)>5):
                ho.linear.x=0
                ho.angular.z=0.3
            else:
                ho.angular.z=0
                ho.linear.x=0.2
            pub.publish(ho)
            rospy.loginfo(ho)


def positcb(msg):
    global x
    global y
    global theta
    global phi
    global thetad
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    ori=msg.pose.pose.orientation
    (alpha,beta,theta)=euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])
    #print("thepose is",x,y,theta)
    thetad=(theta*180)/(math.pi)

#print(maptop(12.02,7.98))

if __name__ == '__main__':
    x=0
    y=0
    theta=0
    gd=0
    path=[]
    ls=0
    tb=[]
    thetad=0
    posi=(x,y)
    ppath =[]

    binsize=5
    binwidth=360/binsize
    print("Hi!")
    astar()
    ppath =[]
    for i in path :
        ppath.append(maptop(i[0],i[1]))
    print("The A star global path is",ppath)
    rospy.init_node('astar_1',anonymous=True)
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    ho=Twist()
    rospy.Subscriber('base_scan',LaserScan,laser)
    rospy.Subscriber('base_pose_ground_truth',Odometry,positcb)
    rospy.Rate(1)
    rospy.sleep(2)


    ##print(ls)
