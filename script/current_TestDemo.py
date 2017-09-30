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

L = 1.0668 #control region, note this is disatnce AFTER S not total distance from the merge which is L+S
S = 0.3048 #merge region
delta = 0.2 #safe distance
mergePosx = 0.7131 #note right direction is negative
mergePosy = 0.7594 #note down is positive
laneWidth = 0.1524 #6"

class Car:
    def __init__(self,name,path,roadID,mergeID,xi,xf,v,vf,a,b,c,inMergeRegion,t,tmi):
        self.name = name
        self.path = path
        self.roadID = roadID #status
        self.mergeID = mergeID
        self.xi = xi
        self.xf = xf
        self.v = v
        self.vf = vf
        self.a = a
        self.b = b
        self.c = c
        self.inMergeRegion = inMergeRegion
        self.t = t
	self.tmi = tmi

car1 = Car('zumoTest1',1,0,0,0,0,0,0,0,0,0,0,0,0)
car2 = Car('zumoTest2',1,0,0,0,0,0,0,0,0,0,0,0,0)
car3 = Car('zumoTest3',1,0,0,0,0,0,0,0,0,0,0,0,0)
car4 = Car('zumoTest4',1,0,0,0,0,0,0,0,0,0,0,0,0)
#car5 = Car('zumoTest5',1,0,0,0,0,0,0,0,0,0,0,0,0)
carList = [car1,car2,car3,car4]#,car5]
numCars = len(carList)

tempFIFO = [0,0,0,0,0] #length has to store at least as many values as number cars
temptm = [0,0,0,0,0]
tempto = [0,0,0,0,0]

def RT_control(to,tmi,xi,xf,v,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tmi**3/6,tmi**2/2,tmi,1],[tmi**2/2,tmi,1,0]])
    Y_mtx = np.matrix([[xi],[v],[xf],[vf]])

    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx

def controller_client(index,v_road,xPos,yPos,lv,rv): #rv,lv is zumo right/left velocity
    global L,S,delta #control region,merge region
    global mergePosx, mergePosy, laneWidth #used to determine if inside control region
    global mergeRegion,FIFO,to,tmi,xi,xf,v,vf,a,b,c,d,xo,vo #dynamic class variables
    global v_target,v_scale,p_target
    global tmi,tfj,tfi,toi,t,t_old,x_old
    global inMergeRegion
    global deltaX

    if path == 1 and xPos>(mergePosx-(L+S)) and (300+mergePosy-laneWidth/2)<yPos<(mergePosy+laneWidth/2) and ((xPos-mergePosx)**2+(yPos-mergePosy)**2)**(1/2)<(L+S): #robot enters control region, last statement accounts for position in lane 2D error
        inMergeRegion = 1
    elif path == 2 and (300+mergePosy-L-S)<yPos<mergePosy and (mergePosx-laneWidth*4)<xPos<mergePosx: 
        inMergeRegion = 1
        #rospy.loginfo("Zumo " +str(index+1)+ " in control region")
    else:
	inMergeRegion =0
    if inMergeRegion ==1:
        if mergeRegion == 0: #when a car first enters the control region calculate a,b,c
            xo = 0 #((xPos-mergePosx)**2+(yPos-mergePosy)**2)**(1/2)
            xf = L
            vo = v_road
            vf = vo
            if max(tempFIFO) == 0: #the first car sets to
                to = time.time() 
                rospy.loginfo("Zumo " +str(index+1)+" to "+str(to))
                tmi = 0 + xf/vf #should be to, but use zero
                tempFIFO[index] = 1 #index is the car number - 1
                temptm[index]=tmi+S/vo #save tmi for first
                tempto[index] = 0 #save tfo for first car
                X_mtx = RT_control(0,tmi,xo,xf,vo,vf) #update controller based on v/p_targets, for next input
		rospy.loginfo(X_mtx)
            elif max(tempFIFO) > 0: #next car the initial time is relative to the FIFO start time
                toi = time.time() - to
                rospy.loginfo("Zumo " +str(index+1)+" toi "+str(toi))
                tempto[index]=toi #save toi
                tfj = temptm[tempFIFO.index(max(tempFIFO))] #tfj is tmi of previous FIFO
                #tfj = tmj + S/vo
                #tfi = tfj + delta/vo
                #tmi = tfi - S/vo
                tfi = tfj + delta/vo
                tmi = tfi - S/vo
		rospy.loginfo("Pineapple (initial control parameters) " +str(index+1)+" toi: "+str(toi)+" / tmi: " +str(tmi)+" / xo: " +str(xo)+" / xf: " +str(xf)+" / vo: " +str(vo)+" / vf: " +str(vf))
                tempFIFO[index]=max(tempFIFO)+1 #N FIFO counting index
                temptm[index]=tmi
                X_mtx = RT_control(toi,tmi,xo,xf,vo,vf) #update controller based on v/p_targets, for next input
            a = X_mtx.item(0)
            b = X_mtx.item(1)
            c = X_mtx.item(2)
            mergeRegion = 1
            rospy.loginfo(str(a) + "ADSGA" + str(b) + "asdgkjas" + str(c))

        elif mergeRegion == 1: #expect a parabolic velocity profile moving towards the merge
            t = time.time() - to
            v_target = 1/2*a*t**2 + b*t +c
            v_scale = 2*v_target/(rv+lv)
            rospy.loginfo("Zumo " +str(index+1)+" t "+str(t)+" tmi " +str(tmi)+" xo " +str(xo)+" xf " +str(xf)+" vo " +str(vo)+" vf " +str(vf)+" target velocity "+ str(v_target)) #send velocity inputs based on the last iteration of controller calculation
        
    else: #when not in control region
        mergeRegion = 0
        tempFIFO[index]=0 #if the car is no longer in the merge rest its FIFO index
        temptm[index]=0
        tempto[index]=0
        rospy.loginfo("Zumo " +str(index+1)+" road velocity "+ str(v_road))
        v_scale = 2*v_road/(rv+lv)

    rv = v_scale*rv
    lv = v_scale*lv    
    cmdZumo(index,str(lv).replace('[',"").replace(']',""),str(rv).replace('[',"").replace(']',"")) #note vel is simply overwritten from the original vel if controlled


def lf_client():
    global x
    global y
    global theta
    global status
    global path
    rospy.wait_for_service('lf_grad')
    try:
        nf = rospy.ServiceProxy('lf_grad', LineFollowing)
        resp = nf(status, x, y)
        if path == 1:
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
                if x>-0.8404: #T135
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

        elif path == 2:
            #Road 2 - starts on the top straight away (S61) and loops CW
 #Road 2 - starts on the top straight away (S61) and loops CW
            if status == 7: #S61
              #rospy.loginfo("Zumo on S61")
              #rospy.loginfo("Zumo position: x->"+str(x)+" y->"+str(y))
              if x<0.46: #2286:
                  status = 8
            elif status == 8: #A58
              #rospy.loginfo("Zumo on A58")
              if y>-0.820: 
                  status = 9
            elif status == 9: #S80
              #rospy.loginfo("Zumo on S80")
              if y>0.3010:
                  status = 10
            elif status == 10: #A94
              #rospy.loginfo("Zumo on A94")
              if x>0.6369: 
                  status = 4
            elif status == 4: #S95/91/89
              #rospy.loginfo("Zumo on S95/91/89")
              if x>1.1963: 
                  status = 5
            elif status == 5: #A92
              #rospy.loginfo("Zumo on A92")
              if y<0.317: 
                 status = 6
            elif status == 6: #A87/84/81
	      #rospy.loginfo("Zumo on 87/84/81")
              if x>2.05: 
                  status = 12
            elif status == 11: #A86
              #rospy.loginfo("Zumo on A86")
              if y<.08 and x>2.3: 
                  status = 13
            elif status == 12: #S88
              #rospy.loginfo("Zumo on S88")
              if y<-0.854: 
                  status = 13
            elif status == 13: #A53
              #rospy.loginfo("Zumo on A53")
              if x<2.00304: 
                 status = 7
        else:
            print "specify path"

        return [resp.res, resp.dx, resp.dy]

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def on_connect():
	print ('connected')
	

def cmdZumo(index,a,b):	
	A = str(a)
	B = str(b)
	velocities = str(index+1)+':'+A + ','+ B+';' #zumos defined in zumo starting at 1
	socketIO.emit('getvelocity', velocities)


def proc():
    global tfl
    global x,y,theta
    global v_road
    global v_scale
    global index
    global path,status,mergeRegion,FIFO,to,tmi,xi,xf,v,vf,a,b,c,d #dynamic class variables
    global inMergeRegion
    global t,tc #tc was changed from t to tc in the #Get car position portion

    v_road = 0.2#m/s #######################################################################

    for i in range(numCars):
        index = i

        #Expand the class variables so we can pass them as globals to various functions
        name = carList[index].name
        path = carList[index].path
        status = carList[index].roadID
        mergeRegion = carList[index].mergeID
        xi = carList[index].xi
        xf = carList[index].xf
        v = carList[index].v
        vf = carList[index].vf
        a = carList[index].a
        b = carList[index].b
        c = carList[index].c
        inMergeRegion = carList[index].inMergeRegion
        t = carList[index].t
	tmi = carList[index].tmi

        #Get Car Position
        zumoID = "/vicon/"+name+"/"+name
        tc = tfl.getLatestCommonTime("/vicon/corner_1/corner_1", zumoID) #top left
        position1, quaternion1 = tfl.lookupTransform("/vicon/corner_1/corner_1", zumoID, tc)
        tc = tfl.getLatestCommonTime("/vicon/corner_2/corner_2", zumoID) #top right
        position2, quaternion2 = tfl.lookupTransform("/vicon/corner_2/corner_2", zumoID, tc)
        tc = tfl.getLatestCommonTime("/vicon/corner_3/corner_3", zumoID) #bottom right
        position3, quaternion3 = tfl.lookupTransform("/vicon/corner_3/corner_3", zumoID, tc)
        tc = tfl.getLatestCommonTime("/vicon/corner_4/corner_4", zumoID) #bottom left
        position4, quaternion4 = tfl.lookupTransform("/vicon/corner_4/corner_4", zumoID, tc)
        x = (position1[0]+position2[0]+position3[0]+position4[0])/4
        y = (position1[1]+position2[1]+position3[1]+position4[1])/4
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)
        theta = euler[2]
        
        #if index == 0:
        #    rospy.loginfo("Zumo " +str(index+1)+ " position: x->"+str(x)+" y->"+str(y))

        #Get Vector Field Values
        res = lf_client()
        if res[0] == 0:
	    tv = res[1]*math.cos(theta)+res[2]*math.sin(theta)
	    tw = 1/d*(res[2]*math.cos(theta)-res[1]*math.sin(theta))
	    rv = tv+0.5*l*tw
	    lv = tv-0.5*l*tw
	    controller_client(index,v_road,x,y,lv,rv) #update controller
        elif res[0]==2:
            cmdZumo(index,0,0)
        else:
	    print "Cannot Run NF."

        #Update class variables
        carList[index].roadID = status
        carList[index].mergeID = mergeRegion
        carList[index].xi = xi
        carList[index].xf = xf
        carList[index].v = v
        carList[index].vf = vf
        carList[index].a = a
        carList[index].b = b
        carList[index].c = c
        carList[index].inMergeRegion = inMergeRegion
        carList[index].t = t
	carList[index].tmi = tmi

def listener():
    global tfl
    rospy.init_node('zumo_go', anonymous=True)
    tfl = tf.TransformListener()
    time.sleep(1)

    while not rospy.is_shutdown():
	time.sleep(0.05)
        proc()

#print("Welcome to the UD Scaled Smart City")

if __name__ == '__main__':
    global s
    socketIO = SocketIO('127.0.0.1', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    
    listener()


