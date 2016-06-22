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
import itertools
from helper_functions import pixel_to_point
import cProfile, pstats, StringIO
from scipy.ndimage.filters import gaussian_filter
import Queue as Q
from copy import deepcopy
import rosbag
from active_perception_controller.srv import positionChange,ppl_positionChange
import os
from motion_planner import MotionPlanner
import helper_functions as fn
from active_perception_controller.msg import Person,PersonArray
from random import shuffle

class example(object):
    def __init__(self):
        self.path = Path()
        self.people = []
        self.goal =None
        self.path_array = []
        self.goal_xy = None
        self.nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)

def experiment_load(directory):
    experiment = []
    for subdir,dirs, files in os.walk(directory+'/traj_data/'):
        for file in files:
            print "FILE",file
            bag = rosbag.Bag(directory+'/traj_data/'+file)
            ex = example()
            for topic, msg, t in bag.read_messages():
                if topic == "robot":
                    ex.path.poses.append(msg)
                    ex.path_array.append(np.array([msg.pose.position.x,msg.pose.position.y]))
                elif topic=="people":
                    ex.people.append(msg)
                elif topic == "goal":
                    ex.goal = msg
                    print msg
                    ex.goal_xy = np.array([msg.pose.position.x,msg.pose.position.y])
            bag.close()
            ex.nbrs.fit(ex.path_array)
            experiment.append(deepcopy(ex))
    return experiment

def experiment_load2(directory):
    experiment = []
    for subdir,dirs, files in os.walk(directory+'/traj_data/'):
        for file in files:
            print "FILE",file
            bag = rosbag.Bag(directory+'/traj_data/'+file)
            ex = example()
            for topic, msg, t in bag.read_messages():
                if topic == "path":
                    ex.path = msg
                    for i in msg.poses:
                        ex.path_array.append(np.array([i.pose.position.x,i.pose.position.y]))
                elif topic=="people":
                    ex.people=msg
                elif topic == "goal":
                    ex.goal = msg
                    ex.goal_xy = np.array([msg.pose.position.x,msg.pose.position.y])
            bag.close()
            ex.nbrs.fit(ex.path_array)
            experiment.append(deepcopy(ex))
    return experiment


class Learner(object):
    def __init__(self):
        self.iterations = rospy.get_param("~iterations", 10)
        self.learning_rate = rospy.get_param("~learning_rate", 0.5)
        self.time_margin_factor = rospy.get_param("~time_margin_factor", 1.0)

        self.cache_size = rospy.get_param("~point_cache_size", 2500)
        self.momentum = 0.2
        self.exp_name = rospy.get_param("~experiment_name", "workshop_data3")
        self.session_name = rospy.get_param("~session_name", "rrt_max_margin")
        self.path = os.path.dirname(os.path.abspath(__file__))
        self.directory = self.path+"/"+self.exp_name
        self.results_dir = self.path+"/results/"+self.session_name+"/"
        fn.make_dir(self.results_dir)
        self.experiment_data = experiment_load2(self.directory)
        #pdb.set_trace()
        self.planner = MotionPlanner()
        self.validation_proportion = rospy.get_param("~validation_proportion", 0.8)
        #self.experiment_data = self.experiment_data[:10]
        # shuffle the experimental data: REMEMBER SHUFFLING IS IN PLACE.
        #shuffle(self.experiment_data)
        self.costlib = self.planner.cost_manager
        self.ppl_pub =  rospy.Publisher("person_poses",PersonArray,queue_size = 10)
        self.baseline_eval = False
        #self.gt_weights = np.array([ 2. ,  0.5,  0. ,  0.3 ,  2. ,  0.5,  0.4 ])
        #fn.pickle_saver({"featureset":"icra2","weights":self.gt_weights},self.directory+"/weights.pkl")
        loaded = fn.pickle_loader(self.directory+"/weights.pkl")
        self.gt_weights = loaded["weights"]
        self.gt_featureset = loaded["feature_set"]
        if self.gt_weights!=None:
            self.baseline_eval = True

        self.write_learning_params(self.results_dir)
        shuffle(self.experiment_data)
        self.pareto_run("2")
        shuffle(self.experiment_data)
        self.single_run("3")
        shuffle(self.experiment_data)
        self.single_run("4")
        shuffle(self.experiment_data)
        self.single_run("5")

    def write_learning_params(self,directory):
        f = open(directory+"readme","w")
        f.write("Learning parameters used: \n------ \n \n")
        f.write("Experiment name:  "+ self.exp_name +"\n")
        f.write("Iteration:  "+ str(self.iterations) +"\n")
        f.write("learning_rate:  "+ str(self.learning_rate) +"\n")
        f.write("validation_proportion:  "+ str(self.validation_proportion) +"\n")
        f.close()        

    def pareto_run(self,name):
        # Pareto front run involves RRT at 2,5,8,10 seconds
        results = {}
        self.planner.planning_time = 2;self.planner.max_planning_time = 2
        self.cache_size = 1400
        results["rrt_2"]= self.learning_loop(self.planner,planner_type="rrtstar")
        results["crrt_2"]=  self.learning_loop(self.planner,planner_type="cached_rrt")

        self.planner.planning_time = 5;self.planner.max_planning_time = 5
        self.cache_size = 2300
        results["rrt_5"]= self.learning_loop(self.planner,planner_type="rrtstar")
        results["crrt_5"]=  self.learning_loop(self.planner,planner_type="cached_rrt")

        self.planner.planning_time = 8;self.planner.max_planning_time = 8
        self.cache_size = 2900
        results["rrt_8"]= self.learning_loop(self.planner,planner_type="rrtstar")
        results["crrt_8"]=  self.learning_loop(self.planner,planner_type="cached_rrt")

        self.planner.planning_time = 10;self.planner.max_planning_time = 10
        self.cache_size = 3300
        results["rrt_10"]= self.learning_loop(self.planner,planner_type="rrtstar")
        results["crrt_10"]= self.learning_loop(self.planner,planner_type="cached_rrt")      

        # Then astar for 0.8
        self.planner.astar_res = 0.8
        results["astar_0.8"]=  self.learning_loop(self.planner,planner_type="astar")
        # astar for 0.4
        self.planner.astar_res = 0.5
        results["astar_0.5"]= self.learning_loop(self.planner,planner_type="astar")

        self.planner.astar_res = 0.3
        results["astar_0.3"]=  self.learning_loop(self.planner,planner_type="astar")

        self.planner.astar_res = 0.2
        results["astar_0.2"]=  self.learning_loop(self.planner,planner_type="astar")
        #results = {"star_0.8":results_astar_08}
        fn.pickle_saver(results,self.results_dir+"results_"+name+".pkl")


    def single_run(self,name):

        results_rrtstar = self.learning_loop(self.planner,planner_type="rrtstar")
        # Then astar for 0.8
        self.planner.astar_res = 0.8
        results_astar_08 = self.learning_loop(self.planner,planner_type="astar")
        # astar for 0.4

        # astar for 0.2
        #self.planner.astar_res = 0.2
        #results_astar_02 = self.learning_loop(self.planner,planner_type="astar")
        # cached RRT star
        results_cached_rrt = self.learning_loop(self.planner,planner_type="cached_rrt")

        self.planner.astar_res = 0.3
        results_astar_03 = self.learning_loop(self.planner,planner_type="astar")
        results = {"rrtstar":results_rrtstar,"astar_0.8":results_astar_08,"astar_0.3":results_astar_03,"cached_rrt":results_cached_rrt}
        #results = {"star_0.8":results_astar_08}
        fn.pickle_saver(results,self.results_dir+"results_"+name+".pkl")


        #self.robot_pose_srv = rospy.ServiceProxy('change_robot_position', positionChange)
        #self.ppl_pose_srv = rospy.ServiceProxy('change_people_position', ppl_positionChange, self.handle_ppl_position_change)
    
    def learning_loop(self,motion_planner,planner_type = "rrtstar"):

        # some time bookkeeping
        if planner_type == "rrtstar" or planner_type == "astar":
            motion_planner.planner = planner_type
            time_to_cache=0
        elif planner_type =="cached_rrt":
            motion_planner.planner = "rrtstar"
            cached_trees = []
            time_to_cache = []

        # Initialise weights.
        self.learner_weights = np.zeros(self.costlib.weights.shape[0])
        self.learner_weights[1] = 1
        self.learner_weights[0] = 1# some cost for distance
        validating = False
        similarity = []
        cost_diff = []
        time_taken = []
        initial_paths = []
        final_paths = []
        all_feature_sums = []
        for n,i in enumerate(self.experiment_data):
            self.ppl_pub.publish(i.people)
            rospy.sleep(0.5)
            i.feature_sum = self.feature_sums(i.path_array,i.goal_xy)
            all_feature_sums.append(i.feature_sum/len(i.path_array))
            if self.baseline_eval == True:
                self.costlib.set_featureset(self.gt_featureset)
                i.gt_feature_sum =self.feature_sums(i.path_array,i.goal_xy)
                i.path_cost = np.dot(self.gt_weights,i.gt_feature_sum)
                self.costlib.set_featureset(self.planner.planning_featureset)
        feature_sum_variance = np.var(np.array(all_feature_sums),axis = 0)
        print "FEATURE SUM VARIANCE",feature_sum_variance


        for iteration in range(self.iterations):
            prev_grad = 0
            print "####################ITERATION",iteration
            iter_similarity = []
            iter_cost_diff = []
            iter_time = []
            iter_grad = np.zeros(self.learner_weights.shape[0])
            self.costlib.weights = self.learner_weights
            self.initial_weights = np.copy(self.learner_weights)
            for n,i in enumerate(self.experiment_data):
                if n>=len(self.experiment_data)*(1-self.validation_proportion):
                    validating = True
                else:
                    validating = False
                print "CHANGING POSITION"
                self.planner.publish_empty_path()
                config_change(i.path.poses[0],i.people)
                rospy.sleep(0.5)
                self.planner._goal = i.goal

                if validating==False and planner_type=="cached_rrt" and iteration==0:
                    tic = time.time()
                    cached_trees.append(motion_planner.make_cached_rrt(motion_planner.sample_goal_bias,points_to_cache=self.cache_size))
                    toc = time.time()
                    time_to_cache.append(toc-tic) 

                if validating==False:
                    self.costlib.ref_path_nn = i.nbrs
                    tic = time.time()
                    if planner_type!="cached_rrt":
                        # if there is a time margin factor it will be used during the learning planning step
                        motion_planner.planning_time*=self.time_margin_factor
                        pose_path_la,array_path_la = motion_planner.plan()
                    else:
                        pose_path_la,array_path_la = motion_planner.plan_cached_rrt(cached_trees[n])
                    toc = time.time()
                    iter_time.append(toc-tic)

                self.costlib.ref_path_nn = None
                # recover time margin factor to normal planning time.
                motion_planner.planning_time*=1/self.time_margin_factor
                pose_path,array_path = motion_planner.plan()

                if iteration ==0:
                    initial_paths.append(array_path)
                elif iteration == self.iterations-1:
                    final_paths.append(array_path)

                rospy.sleep(0.5)

                if validating == False:
                    path_feature_sum_la = self.feature_sums(array_path_la,i.goal_xy)
                    iter_grad+= self.learner_weights*0.00 + (i.feature_sum - path_feature_sum_la)#/feature_sum_variance
                    print "GRADIENT", (i.feature_sum - path_feature_sum_la)#/feature_sum_variance#

                path_feature_sum = self.feature_sums(array_path,i.goal_xy)
                # Baseline evaluation if possible.
                if self.baseline_eval == True:
                    #calculate featuresums based on the ground truth featureset whatever that is
                    self.costlib.set_featureset(self.gt_featureset)
                    gt_feature_sum = self.feature_sums(array_path,i.goal_xy)
                    path_base_cost = np.dot(self.gt_weights,gt_feature_sum)
                    iter_cost_diff.append(path_base_cost - i.path_cost)
                    self.costlib.set_featureset(self.planner.planning_featureset)
                print "COST DIFFERENCE", iter_cost_diff
                          
                print "PATH FEATURE SUM",path_feature_sum
                iter_similarity.append(self.get_similarity(i.path_array,array_path))
            grad = (1-self.momentum)*iter_grad/(len(self.experiment_data)*(1-self.validation_proportion)) + self.momentum*prev_grad
            self.learner_weights = self.learner_weights - self.learning_rate*grad
            prev_grad = grad
            idx = np.where(self.learner_weights<0)
            self.learner_weights[idx]=0
            print "WEIGHTS",self.learner_weights
            print "GRADIENT ", grad
            similarity.append(np.array(iter_similarity))
            cost_diff.append(iter_cost_diff)
            time_taken.append(iter_time)

        print cost_diff
        results = {"similarity":similarity,"cost_diff":cost_diff,
                "weights_final":self.learner_weights,"weights_initial":self.initial_weights,
                 "initial_paths":initial_paths,"final_paths":final_paths,"validation_proportion":self.validation_proportion,"time_to_cache":time_to_cache,"time_per_iter":time_taken}

        return results

    def feature_sums(self,xy_path,goal_xy,interpolation=True):
        # calculates path feature sums.
        if interpolation==True:
            xy_path = interpolate_path(xy_path,resolution=0.2)
        feature_sum = 0
        for i in range(len(xy_path)-1):
            if i ==0:
                f1 = self.costlib.featureset(xy_path[i],goal_xy)
                f2 = self.costlib.featureset(xy_path[i+1],goal_xy)
                feature_sum= self.costlib.edge_feature(f1,f2,xy_path[i],xy_path[i+1])
            else:
                f1 = self.costlib.featureset(xy_path[i],goal_xy)
                f2 = self.costlib.featureset(xy_path[i+1],goal_xy)
                feature_sum+= self.costlib.edge_feature(f1,f2,xy_path[i],xy_path[i+1])
        return feature_sum
   

    def get_similarity(self,example_path,learner_path):
        # inputs are the example and learner path in xy.
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        nbrs.fit(np.array(learner_path))
        (dist, idx) = nbrs.kneighbors(example_path)
        return np.sum(dist)

    def test_similarity(self):
        ex1 = self.experiment_data[0].path_array
        ex2 = self.experiment_data[1].path_array
        sim = self.get_similarity(ex1,ex2)
        return sim
    def test_feature_sum(self):
        self.ppl_pub.publish(self.experiment_data[0].people[0])
        rospy.sleep(0.1)
        fs = self.feature_sums(self.experiment_data[0].path_array,self.experiment_data[0].goal_xy)
        return fs

def interpolate_path(path,resolution=0.2):
    new_path = None
    for n in range(len(path)-1):
        diff = path[n+1] - path[n]
        segments = np.floor(np.linalg.norm(diff)/resolution)

        x_arr = np.linspace(0,diff[0],segments)
        y_arr = np.linspace(0,diff[1],segments)
        inter = path[n] + np.append([x_arr],[y_arr],axis=0).T

        if n==0:
            new_path = inter
        else:
            new_path = np.append(new_path,inter,axis=0)
    return new_path

def config_change(robotPose, personPoses):
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
    rospy.init_node('mmp_learner')
    m = Learner()
    rospy.spin()