#!/usr/bin/env python 
from __future__ import with_statement 

import time
import pdb
import sys
import rospy
import math
import numpy as np
import tf
import actionlib
import copy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import helper_functions as fn
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import LaserScan
from active_perception_controller.srv import positionChange,ppl_positionChange
import rosbag
plt.ion()
class Robot_Position_Manager(object):
    def __init__(self):
        self._vel_sub = rospy.Subscriber('cmd_vel_unfiltered', Twist, self.cb)
        self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vw = np.zeros([1,2])
        self.history_size = 2
        self.w_queue = np.zeros(self.history_size)
        self.v_queue = np.zeros(self.history_size)
        self.weights = np.linspace(0,1,self.history_size)/np.sum(np.linspace(0,1,self.history_size))


    def cb(self,msg):
        self.w_queue =np.concatenate((self.w_queue[1:],np.array([msg.angular.z])))
	self.v_queue =np.concatenate((self.v_queue[1:],np.array([msg.linear.x])))
        print "W before =",msg.angular.z
        w_new = np.sum(self.w_queue*self.weights)
	v_new = np.sum(self.v_queue*self.weights)
	if np.abs(v_new) < 0.03:
		v_new=0.
        msg.angular.z =w_new
	msg.linear.x = v_new
        self._vel_pub.publish(msg)


def listener():
    rospy.init_node('cmd_filter',anonymous=True)
    p = Robot_Position_Manager()
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()
    #data = fn.pickle_loader("collected_data.pkl")
    #pdb.set_trace()

