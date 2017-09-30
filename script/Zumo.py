#!/usr/bin/env python

import rospy
import tf
import math
import time
from threading import Thread, Lock
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace

d=0.05
l=0.09

isRun = True
mutex_tf = Lock()
mutex_socket = Lock()

def on_connect():
	print ('connected')
	

def cmdZumo(i,a,b):
	#global count	
	A = str(a)
	B = str(b)
	velocities = str(i)+':'+A + ','+ B+';'
	#print(velocities)
        mutex_socket.acquire()
	socketIO.emit('getvelocity', velocities)
        mutex_socket.release()
	#count= count+1
	#print (count)
    

def constrain(v):
	if v>2: 
		return 2
	elif v<-2: 
		return -2
	else:3
		return v

def zumoThread(index, frameName, controlRate, mutex_tf):
    global d
    global l
    global isRun
    global tfl
    x = 0
    y = 0
    theta = 0
    status = 0
    while isRun:
        mutex_tf.acquire()
        t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/"+frameName+"/"+frameName)
        position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/"+frameName+"/"+frameName)
        position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/"+frameName+"/"+frameName)
        position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/"+frameName+"/"+frameName)
        position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/"+frameName+"/"+frameName, t)
        mutex_tf.release()
        #print position, quaternion
        x = (position1[0]+position2[0]+position3[0]+position4[0])/4
        y = (position1[1]+position2[1]+position3[1]+position4[1])/4
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)
        theta = euler[2]
        rospy.loginfo("Zumo"+str(index)+" position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta))

        rospy.wait_for_service('lf_grad')
        try:
            nf = rospy.ServiceProxy('lf_grad', LineFollowing)
            res = nf(status, x, y)

            if status == 3:
                if (x-0.3772)*(x-0.3772)+(y+0.2694)*(y+0.2694)<=.1143*.1143:
                    status = 0
            elif status == 0:
                if (x+0.3778)*(x+0.3778)+(y-0.4849)*(y-0.4849)<=.1143*.1143:
                    status = 1
            elif status == 1:
                if (x-0.2694)*(x-0.2694)+(y-0.3772)*(y-0.3772)<=.1143*.1143:
                    status = 2
            elif status == 2:
                if (x+0.4849)*(x+0.4849)+(y+0.3772)*(y+0.3772)<=.1143*.1143:
                    status = 3

            if res[0] == 0:
	        tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	        tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	        rv = tv+0.5*l*tw
	        lv = tv-0.5*l*tw
	        v = constrain(lv/5)
	        w = constrain(rv/5)
	        print "Zumo "+str(index)+" u1->" +str(v) +" u2->" +str(w)
	        cmdZumo(index,v,w)
            elif res[0]==2:
                cmdZumo(index,0,0)
            else:
	        print "Zumo "+str(index)+" Cannot Run NF."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        time.sleep(controlRate)


def listener():
    global tfl

    rospy.init_node('zumo_go', anonymous=True)

    tfl = tf.TransformListener()

    time.sleep(1)

    t1 = Thread(target = zumoThread, args = (1, "zumoTest1", 0.02, mutex_tf))
    t1.start()
    t2 = Thread(target = zumoThread, args = (2, "zumoTest2", 0.02, mutex_tf))
    t2.start()
    t3 = Thread(target = zumoThread, args = (3, "zumoTest3", 0.02, mutex_tf))
    t3.start()
    t4 = Thread(target = zumoThread, args = (4, "zumoTest4", 0.02, mutex_tf))
    t4.start()
    t5 = Thread(target = zumoThread, args = (5, "zumoTest5", 0.02, mutex_tf))
    t5.start()
    
    rospy.spin()

if __name__ == '__main__':
    global s
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    #socketIO.wait()
    listener()
    isRun = False
    time.sleep(1)


