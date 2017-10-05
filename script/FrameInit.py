#!/usr/bin/env python

import rospy
import tf
import time
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('lf_corner')

    print "Please wait"
    tfl = tf.TransformListener()
    time.sleep(2)

    t = tfl.getLatestCommonTime("world","/vicon/corner_1/corner_1")
    position1, quaternion1 = tfl.lookupTransform("/world","/vicon/corner_1/corner_1", t)
    t = tfl.getLatestCommonTime("world","/vicon/corner_2/corner_2")
    position2, quaternion2 = tfl.lookupTransform("/world","/vicon/corner_2/corner_2", t)
    t = tfl.getLatestCommonTime("world","/vicon/corner_3/corner_3")
    position3, quaternion3 = tfl.lookupTransform("/world","/vicon/corner_3/corner_3", t)
    t = tfl.getLatestCommonTime("world","/vicon/corner_4/corner_4")
    position4, quaternion4 = tfl.lookupTransform("/world","/vicon/corner_4/corner_4", t)

    rate = rospy.Rate(50) # 10hz
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform(position1,quaternion1,rospy.Time.now(),"static_corner_1","world")
        br.sendTransform(position2,quaternion2,rospy.Time.now(),"static_corner_2","world")
        br.sendTransform(position3,quaternion3,rospy.Time.now(),"static_corner_3","world")
        br.sendTransform(position4,quaternion4,rospy.Time.now(),"static_corner_4","world")
        rate.sleep()
