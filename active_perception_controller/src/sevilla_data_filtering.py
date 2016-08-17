#!/usr/bin/env python
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from math import *
from nav_msgs.msg import Path,OccupancyGrid
import tf
import pdb
import rospy
import roslib
import time
import numpy as np
import scipy as sp
from StringIO import StringIO
from geometry_msgs.msg._PoseStamped import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
import cProfile, pstats, StringIO
from scipy.ndimage.filters import gaussian_filter
from copy import deepcopy
import rosbag
from active_perception_controller.srv import positionChange,ppl_positionChange
import os
from motion_planner import path_to_pose
import helper_functions as fn
from active_perception_controller.msg import Person,PersonArray
from mmp import example
from mmp import experiment_load_sevilla
import helper_functions as fn
class SevillaFilter(object):
    def __init__(self):
        self.filepath = os.path.dirname(os.path.abspath(__file__))
        self.directory = self.filepath + "/sevilla_test/"
        print self.directory
        self.point_sub = rospy.Subscriber("clicked_point",PointStamped,self.point_cb)
        self.expert_path_pub = rospy.Publisher("expert_path",Path,queue_size = 1)
        self.ppl_pub =  rospy.Publisher("person_poses",PersonArray,queue_size = 10)
        self.experiment_data = experiment_load_sevilla(self.directory)
        ex_dat = []
        good_traj = [0,1,2,4,6,7,9,10,11,16,17,20,22,23,24,25,26,27,28,30,31,34,36,37,38,39,43]
        for i in good_traj:
            ex_dat.append(self.experiment_data[i])
        self.experiment_data = ex_dat
        fn.pickle_saver(ex_dat,self.directory+"filtered.pkl")
        print "DATA LOADED--------------------------------"
        self.visualization_counter=0
    def point_cb(self,msg):
        current_datapoint = self.experiment_data[self.visualization_counter]
        self.ppl_pub.publish(current_datapoint.people)
        self.config_change(current_datapoint.path.poses[0],current_datapoint.people)
        # Publish expert datapoint
        for i in current_datapoint.path.poses:
            i.header.frame_id = "map"
            i.header.stamp = rospy.get_rostime()
        self.expert_path_pub.publish(path_to_pose(current_datapoint.path_array))
        if self.visualization_counter<len(self.experiment_data)-1:
            self.visualization_counter +=1
        else:
            self.visualization_counter=0
    def config_change(self,robotPose, personPoses):
        rospy.wait_for_service('change_robot_position')
        try:
            robot_pos = rospy.ServiceProxy('change_robot_position', positionChange)
            print "HERE"
            robot_pos(robotPose.pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('change_people_position')
        try:
            ppl_pos = rospy.ServiceProxy('change_people_position', ppl_positionChange)
            ppl_pos(personPoses)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__=='__main__':
    rospy.init_node('mmp_evaluator')
    m = SevillaFilter()
    rospy.spin()