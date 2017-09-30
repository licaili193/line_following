#!/usr/bin/env python

import rospy
import tf
import math
import time
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace
import numpy as np

firstCar = 0
to = [1,2]
vo = [3,4]
xo = [5,5]
tFinal = [1,2]
xFinal = [3,4]
vFinal = [5,6]
A = [0,0]
A_mtx = [0,0]
aTrans = [0,0]
Y = [0,0]
Y_mtx = [0,0]
A_aux = [0,0]
A_auxp = [0,0]
X_mtxpart1 = [0,0]
X_mtx = [0,0]


A[firstCar] = [[(math.pow(to[firstCar],3))/6 , (math.pow(to[firstCar],2))/2 , to[firstCar] , 1],[(math.pow(to[firstCar],2))/2 , to[firstCar] , 1 , 0],[(math.pow(tFinal[firstCar],3))/6 , (math.pow(tFinal[firstCar],2))/2 , tFinal[firstCar] , 1],[(math.pow(tFinal[firstCar] , 2))/2 , tFinal[firstCar] , 1 , 0]]
A_mtx[firstCar] = np.matrix(A[firstCar])
aTrans[firstCar] = np.transpose(A_mtx[firstCar])
Y[firstCar] = [[xo[firstCar]],[vo[firstCar]],[xFinal[firstCar]],[vFinal[firstCar]]]
Y_mtx[firstCar] = np.matrix(Y[firstCar])
A_aux[firstCar] = A_mtx[firstCar]*aTrans[firstCar]
A_auxp[firstCar] = np.linalg.pinv(A_aux[firstCar])
X_mtxpart1[firstCar] = A_auxp[firstCar]*A_mtx[firstCar]
X_mtx[firstCar] = X_mtxpart1[firstCar]*Y_mtx[firstCar]

print(X_mtx[firstCar][0])
