#!/usr/bin/env python

import rospy
import tf
import math
import time
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace

x=0.0
y=0.0
theta=0.0
count=0

d=0.05
l=0.09

t = 0
v_target = 0
controlRegion = 0 #0 when not using RTControl, 1 when inside the control region

L = 1 #control region, note this is disatnce AFTER S not total distance from the merge which is L+S
S = 0.4 #merge region
mergePosx = 0.7131 #note right direction is negative
mergePosy = 0.7594 #note down is positive
laneWidth = 0.1524 #6"
status = [0,7] #starting region

totalCars = 2
carData = np.matrix([[0]*totalCars]*14)
carData[0]=[x+1 for x in range(totalCars)] #carNumber @ 0
carData[1]=[1] #status path each car starts on  @ 1
#controlRegion  @ 2
#FIFO order  @ 3
#to  @ 4
#tfi  @ 5
#xo  @ 6
#xf  @ 7
#vo  @ 8
#vf  @ 9
#a  @ 10
#b  @ 11
#c  @ 12
carData[13]=[1,2] #path number  @ 13

def RT_control(to,tfi,xo,xf,vo,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tfi**3/6,tfi**2/2,tfi,1],[tfi**2/2,tfi,1,0]])
    Y_mtx = np.matrix([[xo],[vo],[xf],[vf]])

    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx
    

def controller_client(carData,carNumber,v_road,xPos,yPos,lv,rv): #rv,lv is zumo right/left velocity
    global L,S #control region,merge region
    global mergePosx, mergePosy, laneWidth #used to determine control region, mergePos at merging tangent point
    global to,tfi,xo,xf,vo,vf,t
    global a,b,c
    global v_target,controlRegion,v_scale

    if xPos>(mergePosx-(L+S)) and (mergePosy-laneWidth/2)<yPos<(mergePosy+laneWidth/2) and ((xPos-mergePosx)**2+(yPos-mergePosy)**2)**(1/2)<(L+S): #robot enters control region, last statement accounts for position in lane 2D error
        rospy.loginfo("Zumo in control region")
        #for first car...
        controlRegion = carData.item(2,(carNumber-1))
        if controlRegion == 0:
            to = time.time()/1000
            xo = 0 #((xPos-mergeRosx)**2+(yPos-mergePosy)**2)**(1/2)
            xf = L+S
            vo = v_road
            vf = vo
            tfi = 0 + xf/vf
            X_mtx = RT_control(0,tfi,xo,xf,vo,vf)
            rospy.loginfo(to)
            rospy.loginfo(xo)
            rospy.loginfo(xf)
            rospy.loginfo(vo)
            rospy.loginfo(vf)
            rospy.loginfo(tfi)
            rospy.loginfo(X_mtx)
            a = X_mtx[0]
            b = X_mtx[1]
            c = X_mtx[2]
            carData.itemSet(10,(carNumber-1),a) #save the variables into the carData matrix
            carData.itemSet(11,(carNumber-1),b)
            carData.itemSet(12,(carNumber-1),c)
            controlRegion = 1
            rospy.loginfo("Running Controller")
        elif controlRegion == 1:
            a = carData.item(10,(carNumber-1))
            b = carData.item(11,(carNumber-1))
            c = carData.item(12,(carNumber-1))
            t = time.time()/1000 - to
            v_target = 1/2*a*t**2 + b*t +c
            v_scale = 2*v_target/(rv+lv)
         
        #FEEDBACK CONTROL - Do it later, its a pain to get velocity because you have to save a position variable
        '''#tf - if first car in then final time (tf) is current time plus time to get to end of merge region: tf=t+(abs(xf-x)/vo), else tf = time the previous car exits intersection+time it takes current car tog et through intersection: tf = tflast + S/vo where S is length of merging zone and car enters merging zone at speed vm = vo, also vf = vo since speed is contant through merging zone
        x=(L+S)-((xPos-mergeRosx)**2+(yPos-mergePosy)**2)**(1/2) #x=0 when can enters control region
        xf = L+S #add control region and merging region
        #initial velocity is vo, vf = vo, v is real time velocity for feedback control'''

        
    else:
        controlRegion = 0
        v_scale = 2*v_road/(rv+lv)
    carData.itemset(2,(carNumber-1),controlRegion) #update control region in case it changed
    rv = v_scale*rv
    lv = v_scale*lv    
    print "rv->" +str(rv) +" lv->" +str(lv)
    cmdZumo(carNumber,str(lv).replace('[',"").replace(']',""),str(rv).replace('[',"").replace(']',"")) #note vel is simply overwritten from the original vel if controlled

def lf_client(carNumber,carData):
    global x
    global y
    global theta
    global status
    global path

    path = carData.item(13,(carNumber-1))
    status = carData.item(1,(carNumber-1))

    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status, x, y)
        if path == 1:
        #Road 1 - starts on the bottom straight away (S94) and loops CCW
            if status == 0: #S94
                rospy.loginfo("Zumo on S94")
                rospy.loginfo("Zumo position: x->"+str(x)+" y->"+str(y))
                if x<-0.9506: #(x-0.9506)*(x-0.9506)+(y+1.2176)*(y+1.2176)<=.1143*.1143: #T138
                    rospy.loginfo("Zumo hit T138")
                    status = 1
            elif status == 1: #A106
                rospy.loginfo("Zumo on A106")
                if x<-2.276: #if (x-2.33627131249349)*(x-2.33627131249349)+(y+0.8838327157)*(y+0.8838327157)<=.1143*.1143: #T144
                    status = 2
            elif status == 2: #A105/101/100
                rospy.loginfo("Zumo on A105/101/100")
                if x>-1.45 and y>0.3504: #if (x-1.41231379143055)*(x-1.41231379143055)+(y+0.3504063337)*(y+0.3504063337)<=.1143*.1143: #T139
                    status = 3
            elif status == 3: #A99
                rospy.loginfo("Zumo on A99")
                if x>-0.7104: #if (x-0.634236011592554)*(x-0.634236011592554)+(y+0.7613400118)*(y+0.7613400118)<=.1143*.1143: #T135
                    status = 4
            elif status == 4: #S95/91/89
                rospy.loginfo("Zumo on S95/91/89")
                if x>1.1263: #1.1963: #if (x+1.19634099989586)*(x+1.19634099989586)+(y+0.7586730133)*(y+0.7586730133)<=.1143*.1143: #T127
                    status = 5
            elif status == 5: #A92
                rospy.loginfo("Zumo on A92")
                if x>1.6100: #if (x+1.68493513605503)*(x+1.68493513605503)+(y+0.3472719754)*(y+0.3472719754)<=.1143*.1143: #T120
                    status = 6
            elif status == 6: #A87/84/81
                rospy.loginfo("Zumo on A87/84/81")
                if y>1.2190: #if (x+2.15900048005974)*(x+2.15900048005974)+(y+1.219200165)*(y+1.219200165)<=.1143*.1143: #T125
                    status = 0

        elif path == 2:
            #Road 2 - starts on the top straight away (S61) and loops CW
            if status == 7: #S61
                rospy.loginfo("Zumo on S61")
                rospy.loginfo("Zumo position: x->"+str(x)+" y->"+str(y))
                if x<0.2286:
                    status = 8
            elif status == 8: #A58
                rospy.loginfo("Zumo on A58")
                if y>-0.7612: 
                    status = 9
            elif status == 9: #S80
                rospy.loginfo("Zumo on S80")
                if y>0.3810:
                    status = 10
            elif status == 10: #A94
                rospy.loginfo("Zumo on A94")
                if x>0.7131: 
                    status = 4
            elif status == 4: #S95/91/89
                rospy.loginfo("Zumo on S95/91/89")
                if x>0.7587: 
                    status = 5
            elif status == 5: #A92
                rospy.loginfo("Zumo on A92")
                if y<0.3473: 
                    status = 6
            elif status == 6: #A87/84/81
                rospy.loginfo("Zumo on A87/84/81")
                if x>2.0653: 
                    status = 11
            elif status == 11: #A86
                rospy.loginfo("Zumo on A86")
                if y<-0.2794: 
                    status = 12
            elif status == 12: #S88
                rospy.loginfo("Zumo on S88")
                if y<-0.9144: 
                    status = 13
            elif status == 13: #A53
                rospy.loginfo("Zumo on A53")
                if x<1.9304: 
                    status = 7
        else:
            print "specify path"

        carData.itemset(1,(carNumber-1),status) #update status in case it changed

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

def proc(totalCars,carData):
    global tfl
    global x
    global y
    global theta
    global v_road
    global v_scale
    
    v_road = 0.3 #m/s #######################################################################

    for i in range(1,totalCars):
        #Get Car Position
        zumoID = "/vicon/zumoTest"+str(i)+"/zumoTest"+str(i)
        t = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", zumoID) #top left
        position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", zumoID, t)
        t = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", zumoID) #top right
        position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", zumoID, t)
        t = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", zumoID) #bottom right
        position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", zumoID, t)
        t = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", zumoID) #bottom left
        position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", zumoID, t)
        x = (position1[0]+position2[0]+position3[0]+position4[0])/4
        y = (position1[1]+position2[1]+position3[1]+position4[1])/4
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)
        theta = euler[2]
        #rospy.loginfo("Zumo1 position: x->"+str(x)+" y->"+str(y)+" theta->"+str(theta))

        #Get Vector Field Values
        res = lf_client(i,carData) #UPDATES carData, DO I HAVE TO RETURN CAR DATA OR IS IT OKAY SINCE ITS GLOBAL??
        if res[0] == 0:
	    tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	    tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	    rv = tv+0.5*l*tw
	    lv = tv-0.5*l*tw
	    controller_client(carData,i,v_road,x,y,lv,rv) #update controller
        elif res[0]==2:
            cmdZumo(i,0,0)
        else:
	    print "Cannot Run NF."

    #Below happens after car index loop (might want to output all velocity at once to keep consistent time lag?)


def listener():
    global tfl

    rospy.init_node('zumo_go', anonymous=True)

    tfl = tf.TransformListener()

    time.sleep(1)

    while not rospy.is_shutdown():
	time.sleep(0.05)
        proc(totalCars,carData)

if __name__ == '__main__':
    global s
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    #socketIO.wait()
    listener()

