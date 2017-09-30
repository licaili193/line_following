#!/usr/bin/env python
import numpy as np
import rospy
from numpy.linalg import inv,pinv

'''to=2.0000
tfi=7.4565
xo=-36.5888
xf=100
vo=13.4112
vf=13.4112'''

def RT_control(to,tfi,x,xf,v,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tfi**3/6,tfi**2/2,tfi,1],[tfi**2/2,tfi,1,0]])
    Y_mtx = np.matrix([[x],[v],[xf],[vf]])
    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx

if __name__ == '__main__':
    rospy.init_node('zumo_go', anonymous=True)
    global mergeRegion,FIFO,to,tfi,xo,xf,vo,vf,a,b,c,x,v,to1,tfi1,xo1,xf1,vo1,vf1,a1,b1,c1,x1,v1 #dynamic class variables

    '''to=1505777.30633 #(to must aways start at zero)   #LARGE NUMBERS GIVE AN OVERFLOW ISSUE!
    xo=0.0000
    xf=1.4000
    vo=0.202628534678
    vf=0.202628534678
    tfi=to+xf/vf #to+xf/vf #to+xf/vf=1505784.21552   6.90919'''
    
    rospy.loginfo("ROBOT 1")
    to = 0
    xo = 0
    xf = 1.9155
    vo = 0.25
    vf = vo
    tfi = 1.9155/vf #10.15656

    for t in range(0,14):

        
        #x = 1/6*a*t**3 + 1/2*b*t**2 + c*t + d
	X_mtx = RT_control(to,tfi,xo,xf,vo,vf)
        a = X_mtx.item(0)
        b = X_mtx.item(1)
        c = X_mtx.item(2)
        d = X_mtx.item(3)
	v = 1/2*a*t**2 + b*t + c

        #rospy.loginfo(str(t) + " " + str(v))
	rospy.loginfo(str(v))

    rospy.loginfo("ROBOT 2")
    to1 = .2
    xo1 = 0
    xf1 = 1.9155
    vo1 = 0.25
    vf1 = vo1
    tfi1 = tfi + .6239/vo

    for t in range(0,14):

        
        #x = 1/6*a*t**3 + 1/2*b*t**2 + c*t + d
	X_mtx = RT_control(to1,tfi1,xo1,xf1,vo1,vf1)
        a1 = X_mtx.item(0)
        b1 = X_mtx.item(1)
        c1 = X_mtx.item(2)
        d1 = X_mtx.item(3)
	v1 = 1/2*a1*t**2 + b1*t + c1

        #rospy.loginfo(str(t) + " " + str(v))
	rospy.loginfo(str(v1))

