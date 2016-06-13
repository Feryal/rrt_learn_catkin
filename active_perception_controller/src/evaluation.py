#!/usr/bin/env python
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import *
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path,OccupancyGrid
from active_perception_controller.srv import ActivePerceptionPlan, ActivePerceptionPlanResponse
import tf
import pdb
import rospy
import roslib
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud, ChannelFloat32
from sklearn.neighbors import NearestNeighbors
import time
import threading
import numpy as np
import scipy as sp
import scipy.ndimage
from costlib import Cost_Manager
from StringIO import StringIO
from geometry_msgs.msg._PoseStamped import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
import matplotlib.pyplot as plt
import itertools
from helper_functions import pixel_to_point
import cProfile, pstats, StringIO
from scipy.ndimage.filters import gaussian_filter
from copy import deepcopy
import rosbag
from active_perception_controller.srv import positionChange,ppl_positionChange
import os
from motion_planner import path_to_pose,MotionPlanner
import helper_functions as fn
from active_perception_controller.msg import Person,PersonArray
from mmp import example
from mmp import experiment_load2

class Evaluator(object):
    def __init__(self):
        self.exp_name = rospy.get_param("~experiment_name", "workshop_data3")
        self.session_name = rospy.get_param("~session_name", "learning_final")
        self.filepath = os.path.dirname(os.path.abspath(__file__))
        self.directory = self.filepath+"/"+self.exp_name
        # RESULTS IS A DICT OF:
        # "similarity","cost_diff","weights final","weights initial"
        self.results_dir = self.filepath+"/results/"+self.session_name+"/"

        self.experiment_data = experiment_load2(self.directory)
        self.gt_weights = fn.pickle_loader(self.directory+"/weights.pkl")
        self.results1 = fn.pickle_loader(self.results_dir+"results_1.pkl")
        self.results2 = fn.pickle_loader(self.results_dir+"results_1.pkl")
        self.results3 = fn.pickle_loader(self.results_dir+"results_1.pkl")
       # self.results4 = fn.pickle_loader(self.results_dir+"results_4.pkl")
        self.planner = MotionPlanner()
        self.costlib = self.planner.cost_manager
        #self.results2 = fn.pickle_loader(self.results_dir+"results_rrt_08.pkl")
        #self.results["astar_0.8"] = self.results2["astar_0.8"]
        #self.results["rrtstar"] = self.results2["rrtstar"]
        #fn.pickle_saver(self.results,self.results_dir+"results.pkl") 
        self.plots()
        self.ppl_pub =  rospy.Publisher("person_poses",PersonArray,queue_size = 10)
        self.expert_path_pub = rospy.Publisher("expert_path",Path,queue_size = 1)
        self.initial_paths_pub = rospy.Publisher("initial_weights_path",Path,queue_size = 1)
        self.final_paths_pub = rospy.Publisher("final_weights_path",Path,queue_size = 1)
        self.point_sub = rospy.Subscriber("clicked_point",PointStamped,self.point_cb)
        self.astar_03_path_pub = rospy.Publisher("astar03_path",Path,queue_size = 1)
        self.astar_08_path_pub = rospy.Publisher("astar08_path",Path,queue_size = 1)
        self.crlt_path_pub = rospy.Publisher("crlt_path",Path,queue_size = 1)
        self.visualization_counter = 0

    def get_multiple_runs(self,method):
        train_size = np.array(self.results1[method]["cost_diff"]).shape[1]*(1-self.results1[method]["validation_proportion"])
        cost_diff_val = np.mean(np.array(self.results1[method]["cost_diff"])[:,train_size:],axis=1)
        cost_diff_val = np.vstack([cost_diff_val,np.mean(np.array(self.results2[method]["cost_diff"])[:,train_size:],axis=1)])
        print "WEIGHTS"+method,self.results2[method]["weights_final"]
        cost_diff_val = np.vstack([cost_diff_val,np.mean(np.array(self.results3[method]["cost_diff"])[:,train_size:],axis=1)])
        #cost_diff_val = np.vstack([cost_diff_val,np.mean(np.array(self.results4[method]["cost_diff"])[:,train_size:],axis=1)])

        cost_diff_train = np.mean(np.array(self.results1[method]["cost_diff"])[:,:train_size],axis=1)
        cost_diff_train = np.vstack([cost_diff_train,np.mean(np.array(self.results2[method]["cost_diff"])[:,:train_size],axis=1)])
        cost_diff_train = np.vstack([cost_diff_train,np.mean(np.array(self.results3[method]["cost_diff"])[:,:train_size],axis=1)])
        #cost_diff_train = np.vstack([cost_diff_train,np.mean(np.array(self.results4[method]["cost_diff"])[:,:train_size],axis=1)])

        # return the mean and standard error across runs
        val_mean = np.mean(cost_diff_val,axis=0)
        val_std_err = np.std(cost_diff_val,axis=0)/np.sqrt(cost_diff_val.shape[1])

        train_mean = np.mean(cost_diff_train,axis=0)
        train_std_err = np.std(cost_diff_train,axis=0)/np.sqrt(cost_diff_train.shape[1])

        return train_mean,train_std_err,val_mean,val_std_err

    def plots(self):
        #train_size = np.array(self.results1["rrtstar"]["cost_diff"]).shape[1]*(1-self.results1["rrtstar"][validation_proportion])
        # cost_diff_rrtstar_val = np.mean(np.array(self.results1["rrtstar"]["cost_diff"])[:,train_size:],axis=1)
        # cost_diff_rrtstar_val = np.hstack([cost_diff_rrtstar_val,np.mean(np.array(self.results2["rrtstar"]["cost_diff"])[:,train_size:],axis=1)])
        # cost_diff_rrtstar_val = np.hstack([cost_diff_rrtstar_val,np.mean(np.array(self.results3["rrtstar"]["cost_diff"])[:,train_size:],axis=1)])
        # cost_diff_rrtstar_val = np.hstack([cost_diff_rrtstar_val,np.mean(np.array(self.results3["rrtstar"]["cost_diff"])[:,train_size:],axis=1)])
        rrt_res = self.get_multiple_runs("cached_rrt")
        crrt_res = self.get_multiple_runs("cached_rrt")
        a08_res = self.get_multiple_runs("astar_0.8")
        a03_res = self.get_multiple_runs("astar_0.8")

        # cost_diff_astar08 = np.array(self.results["astar_0.8"]["cost_diff"])[:,:10]
        # cost_diff_astar03 = np.array(self.results["astar_0.3"]["cost_diff"])[:,:10]
        # cost_diff_cached = np.array(self.results["cached_rrt"]["cost_diff"])[:,:10]
        #pdb.set_trace()
        # rows are iterations columns are examples.
        # avg_rrt = np.mean(cost_diff_rrtstar,axis = 1)
        # avg_astar08 = np.mean(cost_diff_astar08,axis = 1)
        # avg_astar03 = np.mean(cost_diff_astar03,axis = 1)
        # #avg_astar02 = np.sum(cost_diff_astar02,axis = 1)/cost_diff_astar02.shape[1]
        # avg_cached = np.mean(cost_diff_cached,axis = 1)

        # med_rrt = np.median(cost_diff_rrtstar,axis = 1)
        # med_astar08 = np.median(cost_diff_astar08,axis = 1)
        # med_astar03 = np.median(cost_diff_astar03,axis = 1)
        # #avg_astar02 = np.sum(cost_diff_astar02,axis = 1)/cost_diff_astar02.shape[1]
        # med_cached = np.median(cost_diff_cached,axis = 1)
        f = plt.figure()
        ax = f.add_subplot(111)

        print "RESULTS HERE",rrt_res[-2][-1],a08_res[-2][-1],a03_res[-2][-1],crrt_res[-2][-1]

        ax.errorbar(range(len(rrt_res[0])),rrt_res[0],label="RLT*-NC",yerr=rrt_res[1])
        ax.errorbar(range(len(a08_res[0])),a08_res[0],label="MMP 0.8m",c="g",yerr=a08_res[1])
        ax.errorbar(range(len(a03_res[0])),a03_res[0],label="MMP 0.3m",c="m",yerr=a03_res[1])
        #ax.plot(avg_astar02,label="A* 0.2m",c="g")
        ax.errorbar(range(len(crrt_res[0])),crrt_res[0],label="RLT*",c="r",yerr=crrt_res[1])
        plt.legend(bbox_to_anchor=(1., 1,0.,-0.06),loc=1)
        ax.set_ylabel("Average Cost difference",fontweight = 'bold',fontsize = 14)
        ax.set_xlabel("Iterations",fontweight='bold',fontsize = 14)
        f.savefig(self.results_dir+"/cost_diff_train.png")

        f = plt.figure()
        ax = f.add_subplot(111)
        ax.errorbar(range(len(rrt_res[2])),rrt_res[2],label="RLT*-NC",yerr=rrt_res[3])
        ax.errorbar(range(len(a08_res[2])),a08_res[2],label="MMP 0.8m",c="g",yerr=a08_res[3])
        ax.errorbar(range(len(a03_res[2])),a03_res[2],label="MMP 0.3m",c="m",yerr=a03_res[3])
        #ax.plot(avg_astar02,label="A* 0.2m",c="g") 
        ax.errorbar(range(len(crrt_res[2])),crrt_res[2],label="RLT*",c="r",yerr=crrt_res[3])
        plt.legend(bbox_to_anchor=(1., 1,0.,-0.06),loc=1)
        ax.set_ylabel("Average Cost difference",fontweight = 'bold',fontsize = 14)
        ax.set_xlabel("Iterations",fontweight='bold',fontsize = 14)
        f.savefig(self.results_dir+"/cost_diff_val.png")


    def point_cb(self,msg):
        current_datapoint = self.experiment_data[self.visualization_counter]
        self.ppl_pub.publish(current_datapoint.people)
        self.config_change(current_datapoint.path.poses[0],current_datapoint.people)
        # Publish expert datapoint
        for i in current_datapoint.path.poses:
            i.header.frame_id = "map"
            i.header.stamp = rospy.get_rostime()

        self.costlib.weights = self.results1["cached_rrt"]["weights_final"]
        #self.costlib.weights = self.gt_weights
        self.expert_path_pub.publish(path_to_pose(current_datapoint.path_array))
        # Plan using initial weights
        self.initial_paths_pub.publish(path_to_pose(self.results1["cached_rrt"]["initial_paths"][self.visualization_counter]))
        self.crlt_path_pub.publish(path_to_pose(self.results1["cached_rrt"]["final_paths"][self.visualization_counter]))
        self.astar_08_path_pub.publish(path_to_pose(self.results1["astar_0.8"]["final_paths"][self.visualization_counter]))
        self.astar_03_path_pub.publish(path_to_pose(self.results1["astar_0.3"]["final_paths"][self.visualization_counter]))
        if self.visualization_counter<len(self.experiment_data)-2:
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
    m = Evaluator()
    rospy.spin()