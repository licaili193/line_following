#!/usr/bin/env python

import rospy
import tf
import math
import time
import copy
import numpy as np
from numpy.linalg import inv,pinv
from threading import Thread, Lock
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace

import Path

d=0.05
l=0.09

isRun = True
mutex_tf = Lock()
mutex_socket = Lock()

def RT_control(to,tmi,xi,xf,v,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tmi**3/6,tmi**2/2,tmi,1],[tmi**2/2,tmi,1,0]])
    Y_mtx = np.matrix([[xi],[v],[xf],[vf]])

    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx

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

def zumoThread(index, frameName, controlRate, mutex_tf, path):
    global d
    global l
    global isRun
    global tfl
    f_size = 30
    f_array = [0]
    for i in range(1,f_size):
        f_array.append(0)
    f_sum = 0
    f_index = 0
    x = 0
    y = 0
    pre_x = 0
    pre_y = 0
    theta = 0
    status = path.GetSegmentIDNoCheck()
    speed = 0.3*1.145
    cur_speed = 0
    p_index = 0

    nf_c = rospy.ServiceProxy('lf_check', CollisionCheck)
    nf_m = rospy.ServiceProxy('lf_merge', MergeControl)
    nf = rospy.ServiceProxy('lf_grad', LineFollowing)

    isControlled = False
    abcd = [0,0,0,0]
    tinit = 0

    controlInfoPre = (None,None)
    cur_t = time.time()

    position1 = None
    quaternion1 = None
    position2 = None
    quaternion2 = None
    position3 = None
    quaternion3 = None
    position4 = None
    quaternion4 = None

    while isRun:
        try:
            mutex_tf.acquire()
            t = tfl.getLatestCommonTime("/static_corner_1", "/vicon/"+frameName+"/"+frameName)
            position1, quaternion1 = tfl.lookupTransform("/static_corner_1", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/static_corner_2", "/vicon/"+frameName+"/"+frameName)
            position2, quaternion2 = tfl.lookupTransform("/static_corner_2", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/static_corner_3", "/vicon/"+frameName+"/"+frameName)
            position3, quaternion3 = tfl.lookupTransform("/static_corner_3", "/vicon/"+frameName+"/"+frameName, t)
            t = tfl.getLatestCommonTime("/static_corner_4", "/vicon/"+frameName+"/"+frameName)
            position4, quaternion4 = tfl.lookupTransform("/static_corner_4", "/vicon/"+frameName+"/"+frameName, t)
            mutex_tf.release()
            #print position, quaternion
        except:
            print "Failed to call tf"
        if (position1 is None) or (quaternion1 is None): continue
        if (position2 is None) or (quaternion2 is None): continue
        if (position3 is None) or (quaternion3 is None): continue
        if (position4 is None) or (quaternion4 is None): continue
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
        #print str(temp_t-cur_t)
        cur_t = temp_t
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)
        theta = euler[2]
        #rospy.loginfo("Zumo"+str(index)+" position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta)+" speed->"+str(cur_speed))
        #rospy.loginfo("Zumo"+str(index)+" speed->"+str(cur_speed)+" status->"+str(status))
        #print "Zumo"+str(index)+" speed->"+str(cur_speed)+" status->"+str(status)
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y)
            res = [resp.res, resp.dx, resp.dy]
            status = path.GetSegmentID(x,y)

            if res[0] == 0:
                rospy.wait_for_service('lf_check')
                cf = nf_c(index, x, y, theta, 0)
                ratio = 1
                if cf.res == 0:
                    ratio = cf.ratio
                else:
                    print "Unable to check collision"

                controlInfo = path.CheckControl(x, y)
                if (controlInfo is not None) and (controlInfo != controlInfoPre):
                    if controlInfo[0] == 0:
                        rospy.wait_for_service('lf_merge')
                        mf = nf_m(index,controlInfo[1],controlInfo[2],0,cur_speed)
                        if mf.isFirst: isControlled = False
                        else:
                            isControlled = True
                            abcd = RT_control(time.time()-mf.tinit,mf.tmi-mf.tinit,0,mf.L,cur_speed,cur_speed)
                            tinit = mf.tinit
                            print "Robot "+str(index)+": to->"+str(time.time()-mf.tinit)+" tmi->"+str(mf.tmi-mf.tinit)+" xi->0 xf->"+str(mf.L)+" v->"+str(cur_speed)+" vf->"+str(cur_speed)
                            print "ABCD: "+str(abcd)
                    elif controlInfo[0] == 2:
                        isControlled = False
                        rospy.wait_for_service('lf_merge')
                        mf = nf_m(index,controlInfo[1],controlInfo[2],1,cur_speed)
                    elif controlInfo[0] == 1:
                        isControlled = False
                controlInfoPre = controlInfo

	        tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	        tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	        rv = tv+0.5*l*tw
	        lv = tv-0.5*l*tw
                diff = abs(rv-lv)
                fac = 0
                if not isControlled: fac = 2*speed/(rv+lv)
                else:
                    temps = 0.5*abcd[0]*(time.time()-tinit)*(time.time()-tinit)+abcd[1]*(time.time()-tinit)+abcd[2]
                    ttemps = temps.item(0)
                    vdiff = ttemps-cur_speed
                    ttemps = ttemps+0.5*vdiff
                    if abcd[0]>0 and ttemps>speed: ttemps = speed
                    if abcd[0]<=0 and ttemps<speed: ttemps = speed 
                    if ttemps<0.1: ttemps = 0.1
                    if ttemps>0.7: ttemps = 0.7
                    #print ttemps
                    fac = 2*ttemps/(rv+lv)
	        v = fac*lv*ratio*(1/(diff+1))
	        w = fac*rv*ratio*(1/(diff+1))
	        #print "Zumo "+str(index)+" u1->" +str(v) +" u2->" +str(w)
	        cmdZumo(index,v,w)
            elif res[0]==2:
                cf = nf_c(index, 0, 0, 0, 1)
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
    '''
    t1 = Thread(target = zumoThread, args = (1, "zumoTest1", 0.1, mutex_tf, copy.deepcopy(Path.GetDefaultPath(0))))
    t1.start()
    t2 = Thread(target = zumoThread, args = (2, "zumoTest2", 0.1, mutex_tf, copy.deepcopy(Path.GetDefaultPath(0))))
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
    '''
    t11 = Thread(target = zumoThread, args = (11, "zumoTest11", 0.01, mutex_tf, copy.deepcopy(Path.GetDefaultPath(0))))
    t11.start()
    t12 = Thread(target = zumoThread, args = (12, "zumoTest12", 0.01, mutex_tf, copy.deepcopy(Path.GetDefaultPath(0))))
    t12.start()
    t13 = Thread(target = zumoThread, args = (13, "zumoTest13", 0.01, mutex_tf, copy.deepcopy(Path.GetDefaultPath(1))))
    t13.start()
    #t14 = Thread(target = zumoThread, args = (14, "zumoTest14", 0.01, mutex_tf, copy.deepcopy(Path.GetDefaultPath(0))))
    #t14.start()
    t15 = Thread(target = zumoThread, args = (15, "zumoTest15", 0.01, mutex_tf, copy.deepcopy(Path.GetDefaultPath(1))))
    t15.start()
    
    rospy.spin()

if __name__ == '__main__':
    global s
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    #socketIO.wait()
    listener()
    isRun = False
    time.sleep(1)


