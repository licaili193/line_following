#!/usr/bin/env python

import rospy
import tf
import math
import time
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace
import numpy as np

x=0.0
y=0.0
theta=0.0
count=0

x2=0.0
y2=0.0
theta2=0.0

x3=0.0
y3=0.0
theta3=0.0

x4=0.0
y4=0.0
theta4=0.0

x5=0.0
y5=0.0
theta5=0.0

d=0.05
l=0.09

status = 3
status2 = 3
status3 = 3
status4 = 3
status5 = 3

bannana = [-1,-1]
xo = [0,0]
vo = [0,0]
to = [0,0]
xFinal = [0,0]
vFinal = [0,0]
tFinal = [0,0]
A = [0,0]
A_mtx = [0,0]
Y = [0,0]
Y_mtx = [0,0]
aTrans = [0,0]
A_aux = [0,0]
X_mtx = [0,0]
A_auxp = [0,0]
X_mtxpart1 = [0,0]
leftWheel = [0.00,0.00]
rightWheel = [0.00,0.00]

xPos = 0
xPos2 = 0
yPos = 0 
yPos2 = 0
lineVel = 0
lineVel2 = 0
cz_length = .8
mz_length = .3
secondCar = 50
firstCar = 50
distance = 1
distance2 = 1
region =0 
xApple = [1,1]


def controller_client(*arg):
	global xPos
	global yPos		
	global xPos2
	global yPos2
	global lineVel
	global lineVel2
	global rightWheel
	global leftWheel
	global rightWheel2
	global leftWheel2
	global clientInput 
	global cz_length
	global mz_length
	global region
	global scale
	global secondCar
	global firstCar
	global distance
	global distance2
	#Set i to 0 to Start and Add 1 for Each Vehicle's Entry
	i=0
	#Scaling Factor To Generalize Speed and Keep Line Following Accurate
	scale = 5
	clientInput = arg[:6]
	rospy.loginfo(clientInput[0])
	if clientInput[0] == 2:
		lineVel2 = clientInput[1]
		xPos2 = clientInput[2]
		yPos2= clientInput[3]
		leftWheel[1] = clientInput[4]
		rightWheel[1] = clientInput[5]
		distance2 = math.sqrt((xPos2*xPos2)+(yPos2*yPos2))
	if clientInput[0] == 1:
		lineVel = clientInput[1]
		xPos = clientInput[2]
		yPos = clientInput[3]
		leftWheel[0] = clientInput[4]
		rightWheel[0] = clientInput[5]
		distance = math.sqrt((xPos*xPos)+(yPos*yPos))
	if distance2 < .8:
		bannana[1] = min(bannana)-1
		xo[1] = math.sqrt((xPos2*xPos2)+(yPos2*yPos2)) 
		vo[1] = lineVel2
		to[1] = time.time()/1000
	else:
		cmdZumo(2,leftWheel[1]/5,rightWheel[1]/5)
	if distance < .8: 
		bannana[0] = min(bannana)-1
		xo[0] = ((xPos*xPos)+(yPos*yPos))
		vo[0] = lineVel
		to[0] = time.time()/1000
	else:
		cmdZumo(1,leftWheel[0]/5,rightWheel[0]/5)
	if distance > .8 and distance2 > .8:
		region = 1
	firstCar = bannana.index(min(bannana))
	if min(bannana) != -1:
		if region == 1:
			region = 0
			tFinal[firstCar] = 1/(vo[firstCar]/(cz_length-mz_length))/1000
			vFinal[firstCar] = vo[firstCar]
			xFinal[firstCar] = cz_length + mz_length
			bannana[firstCar] = -1
			if min(bannana) != -1:
				secondCar = bannana.index(max(bannana))
				bannana[secondCar] = -1
				tFinal[secondCar] = 1/(tFinal[firstCar]+vFinal[firstCar]/mz_length)/1000
				vFinal[secondCar] = vFinal[firstCar]
				xFinal[secondCar] = cz_length+mz_length
				A[secondCar] = [[(math.pow(to[secondCar],3))/6 , (math.pow(to[secondCar],2))/2 , to[secondCar] , 1],[(math.pow(to[secondCar],2))/2 , to[secondCar] , 1 , 0],[(math.pow(tFinal[secondCar],3))/6 , (math.pow(tFinal[secondCar],2))/2 , tFinal[secondCar] , 1],[(math.pow(tFinal[secondCar] , 2))/2 , tFinal[secondCar] , 1 , 0]]
				A_mtx[secondCar] = np.matrix(A[secondCar])
				aTrans[secondCar] = np.linalg.inv(A_mtx[secondCar])
				Y[secondCar] = [[xo[secondCar]],[vo[secondCar]],[xFinal[secondCar]],[vFinal[secondCar]]]
				Y_mtx[secondCar] = np.matrix(Y[secondCar])
				A_aux[secondCar] = np.dot(A_mtx[secondCar],Y_mtx[secondCar])
				A_auxp[secondCar] = np.linalg.pinv(A_aux[secondCar])
				X_mtxpart1[secondCar] = np.dot(A_auxp[secondCar],A_mtx[secondCar])
				X_mtx[secondCar] = np.dot(X_mtxpart1[secondCar],Y_mtx[secondCar])

		A[firstCar] = [[(math.pow(to[firstCar],3))/6 , (math.pow(to[firstCar],2))/2 , to[firstCar] , 1],[(math.pow(to[firstCar],2))/2 , to[firstCar] , 1 , 0],[(math.pow(tFinal[firstCar],3))/6 , (math.pow(tFinal[firstCar],2))/2 , tFinal[firstCar] , 1],[(math.pow(tFinal[firstCar] , 2))/2 , tFinal[firstCar] , 1 , 0]]
		A_mtx[firstCar] = np.matrix(A[firstCar])
		aTrans[firstCar] = np.transpose(A_mtx[firstCar])
		Y[firstCar] = [[xo[firstCar]],[vo[firstCar]],[xFinal[firstCar]],[vFinal[firstCar]]]
		Y_mtx[firstCar] = np.matrix(Y[firstCar])
		A_aux[firstCar] = A_mtx[firstCar]*aTrans[firstCar]
		A_auxp[firstCar] = np.linalg.pinv(A_aux[firstCar])
		X_mtxpart1[firstCar] = A_auxp[firstCar]*A_mtx[firstCar]
		X_mtx[firstCar] = X_mtxpart1[firstCar]*Y_mtx[firstCar]
		rospy.loginfo(X_mtx[firstCar][0])
		rospy.loginfo(X_mtx[firstCar][1])
		rospy.loginfo(X_mtx[firstCar][2])
        
		xApple[firstCar] = (math.pow(to[firstCar],2))*X_mtx[firstCar][0]+to[firstCar]*X_mtx[firstCar][1]+X_mtx[firstCar][2]
		rospy.loginfo(xApple[firstCar])
		cmdZumo(firstCar+1,leftWheel[firstCar]*xApple[firstCar],rightWheel[firstCar]*xApple[firstCar])

	
			
def lf_client():
    global x
    global y
    global theta

    global status

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status, x, y)

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

        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def lf_client2():
    global x2
    global y2
    global theta2

    global status2

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status2, x2, y2)

        if status2 == 3:
            if (x2-0.3772)*(x2-0.3772)+(y2+0.2694)*(y2+0.2694)<=.1143*.1143:
                status2 = 0
        elif status2 == 0:
            if (x2+0.3778)*(x2+0.3778)+(y2-0.4849)*(y2-0.4849)<=.1143*.1143:
                status2 = 1
        elif status2 == 1:
            if (x2-0.2694)*(x2-0.2694)+(y2-0.3772)*(y2-0.3772)<=.1143*.1143:
                status2 = 2
        elif status2 == 2:
            if (x2+0.4849)*(x2+0.4849)+(y2+0.3772)*(y2+0.3772)<=.1143*.1143:
                status2 = 3

        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def lf_client3():
    global x3
    global y3
    global theta3

    global status3

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status3, x3, y3)

        if status3 == 3:
            if (x3-0.3772)*(x3-0.3772)+(y3+0.2694)*(y3+0.2694)<=.1143*.1143:
                status3 = 0
        elif status3 == 0:
            if (x3+0.3778)*(x3+0.3778)+(y3-0.4849)*(y3-0.4849)<=.1143*.1143:
                status3 = 1
        elif status3 == 1:
            if (x3-0.2694)*(x3-0.2694)+(y3-0.3772)*(y3-0.3772)<=.1143*.1143:
                status3 = 2
        elif status3 == 2:
            if (x3+0.4849)*(x3+0.4849)+(y3+0.3772)*(y3+0.3772)<=.1143*.1143:
                status3 = 3

        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def lf_client4():
    global x4
    global y4
    global theta4

    global status4

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status4, x4, y4)

        if status4 == 3:
            if (x4-0.3772)*(x4-0.3772)+(y4+0.2694)*(y4+0.2694)<=.1143*.1143:
                status4 = 0
        elif status4 == 0:
            if (x4+0.3778)*(x4+0.3778)+(y4-0.4849)*(y4-0.4849)<=.1143*.1143:
                status4 = 1
        elif status4 == 1:
            if (x4-0.2694)*(x4-0.2694)+(y4-0.3772)*(y4-0.3772)<=.1143*.1143:
                status4 = 2
        elif status4 == 2:
            if (x4+0.4849)*(x4+0.4849)+(y4+0.3772)*(y4+0.3772)<=.1143*.1143:
                status4 = 3

        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def lf_client5():
    global x5
    global y5
    global theta5

    global status5

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status5, x5, y5)

        if status5 == 3:
            if (x5-0.3772)*(x5-0.3772)+(y5+0.2694)*(y5+0.2694)<=.1143*.1143:
                status5 = 0
        elif status5 == 0:
            if (x5+0.3778)*(x5+0.3778)+(y5-0.4849)*(y5-0.4849)<=.1143*.1143:
                status5 = 1
        elif status5 == 1:
            if (x5-0.2694)*(x5-0.2694)+(y5-0.3772)*(y5-0.3772)<=.1143*.1143:
                status5 = 2
        elif status5 == 2:
            if (x5+0.4849)*(x5+0.4849)+(y5+0.3772)*(y5+0.3772)<=.1143*.1143:
                status5 = 3

        return [resp.res, resp.dx, resp.dy]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def on_connect():
	print ('connected')
	

def cmdZumo(i,a,b):
	global count	
	A = str(a)
	B = str(b)
	velocities = str(i)+':'+A + ','+ B+';'
	#print(velocities)
	socketIO.emit('getvelocity', velocities)
	count= count+1
	#print (count)
    

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
    global x2
    global y2
    global theta2
    global x3
    global y3
    global theta3
    global x4
    global y4
    global theta4
    global x5
    global y5 
    global theta5

    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTest/zumoTest")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTest/zumoTest", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTest/zumoTest")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTest/zumoTest", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTest/zumoTest")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTest/zumoTest", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTest/zumoTest")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTest/zumoTest", t)
    #print position, quaternion
    x = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta = euler[2]
    rospy.loginfo("Zumo1 position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta))

    res = lf_client()
    if res[0] == 0:
	tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = (rv+lv)/2
	controller_client(2,v,x,y,lv,rv)
	print "u1->" +str(rv) +" u2->" +str(lv)
    elif res[0]==2:
        cmdZumo(2,0,0)
    else:
	print "Cannot Run NF."


    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTest2/zumoTest2")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTest2/zumoTest2", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTest2/zumoTest2")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTest2/zumoTest2", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTest2/zumoTest2")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTest2/zumoTest2", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTest2/zumoTest2")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTest2/zumoTest2", t)
    #print position, quaternion
    x2 = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y2 = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta2 = euler[2]
    rospy.loginfo("Zumo2 position: x->"+str(x2)+" y->"+str(y2)+" theta->"+str(theta2))
    
    res = lf_client2()
    if res[0] == 0:
	tv = res[1]*math.cos(theta2)+res[2]*math.sin(theta2)
	tw = 1/d*(res[2]*math.cos(theta2)-res[1]*math.sin(theta2))
	rv = tv+0.5*l*tw
	lv= tv-0.5*l*tw
	v = (rv+lv)/2
	controller_client(1,v,x2,y2,lv,rv)
	print "u1->" +str(rv) +" u2->" +str(lv)
    elif res[0]==2:
        cmdZumo(1,0,0)
    else:
	print "Cannot Run NF."
'''

    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTest3/zumoTest3")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTest3/zumoTest3", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTest3/zumoTest3")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTest3/zumoTest3", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTest3/zumoTest3")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTest3/zumoTest3", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTest3/zumoTest3")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTest3/zumoTest3", t)
    #print position, quaternion
    x3 = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y3 = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta3 = euler[2]
    rospy.loginfo("Zumo3 position: x->"+str(x3)+" y->"+str(y3)+" theta->"+str(theta3))

    res = lf_client3()
    if res[0] == 0:
	tv = res[1]*math.cos(theta3)+res[2]*math.sin(theta3)
	tw = 1/d*(res[2]*math.cos(theta3)-res[1]*math.sin(theta3))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = constrain(lv/5)
	w = constrain(rv/5)
	print "u1->" +str(v) +" u2->" +str(w)
	cmdZumo(3,v,w)
    elif res[0]==2:
        cmdZumo(3,0,0)
    else:
	print "Cannot Run NF."

    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTest4/zumoTest4")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTest4/zumoTest4", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTest4/zumoTest4")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTest4/zumoTest4", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTest4/zumoTest4")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTest4/zumoTest4", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTest4/zumoTest4")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTest4/zumoTest4", t)
    #print position, quaternion
    x4 = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y4 = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta4 = euler[2]
    rospy.loginfo("Zumo4 position: x->"+str(x4)+" y->"+str(y4)+" theta->"+str(theta4))

    res = lf_client4()
    if res[0] == 0:
	tv = res[1]*math.cos(theta4)+res[2]*math.sin(theta4)
	tw = 1/d*(res[2]*math.cos(theta4)-res[1]*math.sin(theta4))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = constrain(lv/5)
	w = constrain(rv/5)
	print "u1->" +str(v) +" u2->" +str(w)
	cmdZumo(4,v,w)
    elif res[0]==2:
        cmdZumo(4,0,0)
    else:
	print "Cannot Run NF."

    t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", "/vicon/zumoTest5/zumoTest5")
    position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", "/vicon/zumoTest5/zumoTest5", t)
    t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", "/vicon/zumoTest5/zumoTest5")
    position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", "/vicon/zumoTest5/zumoTest5", t)
    t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", "/vicon/zumoTest5/zumoTest5")
    position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", "/vicon/zumoTest5/zumoTest5", t)
    t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", "/vicon/zumoTest5/zumoTest5")
    position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", "/vicon/zumoTest5/zumoTest5", t)
    #print position, quaternion
    x5 = (position1[0]+position2[0]+position3[0]+position4[0])/4
    y5 = (position1[1]+position2[1]+position3[1]+position4[1])/4
    q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
    euler = tf.transformations.euler_from_quaternion(q)
    theta5 = euler[2]
    rospy.loginfo("Zumo5 position: x->"+str(x5)+" y->"+str(y5)+" theta->"+str(theta5))

    res = lf_client5()
    if res[0] == 0:
	tv = res[1]*math.cos(theta5)+res[2]*math.sin(theta5)
	tw = 1/d*(res[2]*math.cos(theta5)-res[1]*math.sin(theta5))
	rv = tv+0.5*l*tw
	lv = tv-0.5*l*tw
	v = constrain(lv/5)
	w = constrain(rv/5)
	print "u1->" +str(v) +" u2->" +str(w)
	cmdZumo(5,v,w)
    elif res[0]==2:
        cmdZumo(5,0,0)
    else:
	print "Cannot Run NF."
'''

def listener():
    global tfl

    rospy.init_node('zumo_go', anonymous=True)

    tfl = tf.TransformListener()

    time.sleep(1)

    while not rospy.is_shutdown():
	time.sleep(0.05)
        proc()

if __name__ == '__main__':
    global s
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    #socketIO.wait()
    listener()

