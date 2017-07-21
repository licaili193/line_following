#!/usr/bin/env python

import rospy
import tf
import math
import time
import serial
from geometry_msgs.msg import TransformStamped
from line_following.srv import *

x=0.0
y=0.0
theta=0.0

d=0.1
l=0.085

status = 0

zumo = serial.Serial('/dev/ttyUSB0',115200)
time.sleep(1)

def lf_client():
    global x
    global y
    global theta

    global status

    if status == 0:
        if (x+0.3778)*(x+0.3778)+(y-0.4849)*(y-0.4849)<=0.0762*0.0762:
            status = 1
    elif status == 1:
        if (x-0.2694)*(x-0.2694)+(y-0.3772)*(y-0.3772)<=0.0762*0.0762:
            status = 2
    elif status == 2:
        if (x+0.4849)*(x+0.4849)+(y+0.3772)*(y+0.3772)<=0.0762*0.0762:
            status = 3
    elif status == 3:
        if (x-0.3772)*(x-0.3772)+(y+0.2694)*(y+0.2694)<=0.0762*0.0762:
            status = 0

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status, x, y)
        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def cmdZumo(a,b):
    print zumo.write(str(a)+","+str(b)+";")

def constrain(v):
	if v>2: 
		return 2
	elif v<-2: 
		return -2
	else:
		return v

def proc():
    global tfl
    global x
    global y
    global theta

    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTracks/zumoTracks")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTracks/zumoTracks", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTracks/zumoTracks")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTracks/zumoTracks", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTracks/zumoTracks")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTracks/zumoTracks", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTracks/zumoTracks")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTracks/zumoTracks", t)
    #print position, quaternion
    x = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta = euler[2]
    rospy.loginfo("Zumo position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta))
    
    res = lf_client()
    if res[0] == 0:
	tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = constrain(lv/2)
	w = constrain(rv/2)
	print "u1->" +str(v) +" u2->" +str(w)
	cmdZumo(v,w)
    elif res[0]==2:
        cmdZumo(0,0)
    else:
	print "Cannot Run NF."

def listener():
    global tfl

    rospy.init_node('zumo_go', anonymous=True)

    tfl = tf.TransformListener()

    time.sleep(1)

    while not rospy.is_shutdown():
	time.sleep(0.02)
        proc()

if __name__ == '__main__':
    listener()

