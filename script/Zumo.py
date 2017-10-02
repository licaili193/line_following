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
    else:
        return v

def zumoThread(index, frameName, controlRate, mutex_tf):
    global d
    global l
    global isRun
    global tfl
    f_size = 30
    f_array = [0]*f_size
    f_sum = 0
    f_index = 0
    x = 0
    y = 0
    pre_x = 0
    pre_y = 0
    theta = 0
    status = 0
    speed = 0.3*1.145
    cur_speed = 0
    cur_t = time.time()
    while isRun:
        mutex_tf.acquire()
        try:
            t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/"+frameName+"/"+frameName)
            position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/"+frameName+"/"+frameName)
            position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/"+frameName+"/"+frameName)
            position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/"+frameName+"/"+frameName)
            position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/"+frameName+"/"+frameName, t)
        except e:
            pass
        mutex_tf.release()
        #print position, quaternion
        x = (position1[0]+position2[0]+position3[0]+position4[0])/4
        y = (position1[1]+position2[1]+position3[1]+position4[1])/4
        temp_t = time.time()
        if temp_t != cur_t:
            f_sum-=f_array[f_index]
            f_array[f_index] = math.sqrt((x-pre_x)*(x-pre_x)+(y-pre_y)*(y-pre_y))/(temp_t-cur_t)
            f_sum+=f_array[f_index]
            cur_speed=f_sum/f_size
            f_index = f_index+1
            if f_index>=f_size: f_index = 0
        else:
            cur_speed = -1
        pre_x = x
        pre_y = y
        cur_t = temp_t
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)
        theta = euler[2]
        #rospy.loginfo("Zumo"+str(index)+" position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta)+" speed->"+str(cur_speed))
        rospy.loginfo("Zumo"+str(index)+" speed->"+str(cur_speed))
        rospy.wait_for_service('lf_grad')
        try:
            nf = rospy.ServiceProxy('lf_grad', LineFollowing)
            resp = nf(status, x, y)
            res = [resp.res, resp.dx, resp.dy]

             #Road 1 - starts on the bottom straight away (S94) and loops CCW
            if status == 0: #S94
                #rospy.loginfo("Zumo on S94")
                if x<-0.9506: #T138
                    status = 1
            elif status == 1: #A106
                #rospy.loginfo("Zumo on A106")
                if x<-2.276: #T144
                    status = 2
            elif status == 2: #A105/101/100
                #rospy.loginfo("Zumo on A105/101/100")
                if x>-1.61 and y>0.2042: #T139
                    status = 3
            elif status == 3: #A99
                #rospy.loginfo("Zumo on A99")
                if x>-0.8744:#-0.8404: #T135
                    status = 4
            elif status == 4: #S95/91/89
                #rospy.loginfo("Zumo on S95/91/89")
                if x>1.1263: #T127
                    status = 5
            elif status == 5: #A92
                #rospy.loginfo("Zumo on A92")
                if y<0.435: #T120
                    status = 14
            elif status == 14: #A87
                #rospy.loginfo("Zumo on A87")
                if x>2.2352: #T125
                    status = 15
            elif status == 15: #A84
                #rospy.loginfo("Zumo on A84")
                if y>1.2: #T125
                    status = 6
            elif status == 6: #A81
                #rospy.loginfo("Zumo on A81")
                if x<2.26: #T125
                    status = 0

            if res[0] == 0:
	        tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	        tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	        rv = tv+0.5*l*tw
	        lv = tv-0.5*l*tw
                fac = 2*speed/(rv+lv)
	        v = fac*lv
	        w = fac*rv
                #diff = abs(v-w)
	        #print "Zumo "+str(index)+" u1->" +str(v) +" u2->" +str(w)
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

    t1 = Thread(target = zumoThread, args = (1, "zumoTest1", 0.1, mutex_tf))
    t1.start()
    t2 = Thread(target = zumoThread, args = (2, "zumoTest2", 0.1, mutex_tf))
    t2.start()
    t3 = Thread(target = zumoThread, args = (3, "zumoTest3", 0.1, mutex_tf))
    t3.start()
    t4 = Thread(target = zumoThread, args = (4, "zumoTest4", 0.1, mutex_tf))
    t4.start()
    t5 = Thread(target = zumoThread, args = (5, "zumoTest5", 0.1, mutex_tf))
    t5.start()
    t6 = Thread(target = zumoThread, args = (6, "zumoTest6", 0.1, mutex_tf))
    t6.start()
    t7 = Thread(target = zumoThread, args = (7, "zumoTest7", 0.1, mutex_tf))
    t7.start()
    t8 = Thread(target = zumoThread, args = (8, "zumoTest8", 0.1, mutex_tf))
    t8.start()
    t9 = Thread(target = zumoThread, args = (9, "zumoTest9", 0.1, mutex_tf))
    t9.start()
    t10 = Thread(target = zumoThread, args = (10, "zumoTest10", 0.1, mutex_tf))
    t10.start()
    t11 = Thread(target = zumoThread, args = (11, "zumoTest11", 0.1, mutex_tf))
    t11.start()

    rospy.spin()

if __name__ == '__main__':
    global s
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    #socketIO.wait()
    listener()
    isRun = False
    time.sleep(1)


