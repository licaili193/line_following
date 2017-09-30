#!/usr/bin/env python
import numpy as np
import rospy
from numpy.linalg import inv,pinv

to=2.0000
tf=7.4565
xo=-36.5888
xf=100
vo=13.4112
vf=13.4112

#For these inputs a=-4.6838,b=22.1463,c=-21.5137,d=-31.6088 so X_mtx = [a,b,c,d]

#tf = to ** 2 #you can use ** for power

A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tf**3/6,tf**2/2,tf,1],[tf**2/2,tf,1,0]])
Y_mtx = np.matrix([[xo],[vo],[xf],[vf]])

A_aux = np.transpose(A_mtx)*A_mtx
X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

if __name__ == '__main__':
    rospy.init_node('zumo_go', anonymous=True)
    rospy.loginfo(X_mtx)



