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

zumo = serial.Serial('/dev/ttyUSB0',9600)
time.sleep(1)

def lf_client():
    global x
    global y
    global theta

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(0, x, y)
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

    t = tfl.getLatestCommonTime("/vicon/mapOrigin/mapOrigin", "/vicon/zumoTracks/zumoTracks")
    position, quaternion = tfl.lookupTransform("/vicon/mapOrigin/mapOrigin", "/vicon/zumoTracks/zumoTracks", t)
    #print position, quaternion
    x = position[0]
    y = position[1]
    q = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    euler = tf.transformations.euler_from_quaternion(q)
    theta = euler[2]
    rospy.loginfo("Zumo position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta))
    
    res = lf_client()
    if res[0] == 0:
	tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = constrain(lv/1.75)
	w = constrain(rv/1.75)
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

    while not rospy.is_shutdown():
	time.sleep(0.02)
        proc()

if __name__ == '__main__':
    listener()

