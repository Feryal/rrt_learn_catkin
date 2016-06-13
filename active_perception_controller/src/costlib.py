#!/usr/bin/env python

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import *
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
import tf
import pdb
import rospy
import roslib
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud, ChannelFloat32
from sklearn.neighbors import NearestNeighbors
from active_perception_controller.msg import Person,PersonArray
import time
import threading
import numpy as np
import scipy as sp
import scipy.ndimage
from StringIO import StringIO
from geometry_msgs.msg._PoseStamped import PoseStamped
import math

class Cost_Manager(object):
	def __init__(self,dist_trans,nav_map,features = "icra1",max_plan_dist=30):
		self.distance_transform=dist_trans
		self.nav_map=nav_map
		self.distance_dict = {"linear":linear,"exponential":np.exp,"log":safe_log,
								"one_over":one_over}
		# self.weights = np.ones(7)
		# self.weights[0] = 3.
		# self.weights[1] = 1.
		# self.weights[2] = 0
		# self.weights[-1]=2
		# self.weights[-2]=1
		# self.weights[-3]=7
		# self.weights[-4]=2
		self.weights = np.ones(7)
		self.weights[0] = 2.
		self.weights[1] = 0.5
		self.weights[2] = 0.
		self.weights[3]=0.3
		self.weights[4]=2.
		self.weights[5]=0.5
		self.weights[6]=0.4
		self.max_plan_dist = max_plan_dist
		self.set_featureset(features)
		self.people_sub = rospy.Subscriber("person_poses",
                                                      PersonArray,
                                                      self.ppl_cb,
                                                queue_size=1)
		self.people_latest = None
		self.origin = np.array([self.nav_map.info.origin.position.x,self.nav_map.info.origin.position.y])
		self.res = self.nav_map.info.resolution
		self.ref_path_nn =None

	def ppl_cb(self,msg):
		self.people_latest = msg
		self.simple_ppl_poses = []
		for i in self.people_latest.persons:
			quat = [i.pose.orientation.x,i.pose.orientation.y,i.pose.orientation.z,i.pose.orientation.w]
			ang = tf.transformations.euler_from_quaternion(quat)[2]
			self.simple_ppl_poses.append(np.array([i.pose.position.x,i.pose.position.y,ang]))
		self.simple_ppl_poses = np.array(self.simple_ppl_poses)
	def obstacle_dist(self,robot_xy):
		dt_idx =  idx = np.array([int((robot_xy[0] - self.origin[0])/self.res),int((-robot_xy[1] + self.origin[1])/self.res)])
		return self.distance_transform[dt_idx[1],dt_idx[0]]*self.res
	def goal_cost(self,robot_xy,goal_xy):
		return np.exp(np.linalg.norm(robot_xy- goal_xy))
	def icra_featureset_1(self,robot_xy,goal_xy):
		goal_dst = np.linalg.norm(robot_xy- goal_xy)
		goal_f1 = self.distance_dict["linear"](goal_dst)
		goal_f2 = self.distance_dict["exponential"](goal_dst)
		goal_f3 = self.distance_dict["log"](goal_dst)

		ppl_f1 = 0;ppl_f2=0;ppl_f3=0
		if self.people_latest!=None:
			dist =to_person_frame(robot_xy,self.simple_ppl_poses)
			self.angled_step_feature(dist,0,0)
			ppl_f1= self.gaussian_feature(dist,mean = self.mean1,cov = self.cov1)
			ppl_f2= self.gaussian_feature(dist,mean = self.mean2,cov = self.cov2)
			ppl_f3= self.gaussian_feature(dist,mean = self.mean3,cov = self.cov3)
		obstacle_dist = self.obstacle_dist(robot_xy)
		obs_f1 = 1/(0.01+obstacle_dist)
		return np.array([goal_f1,goal_f2,goal_f3,ppl_f1,ppl_f2,ppl_f3,obs_f1])/self.normaliser
	def icra_featureset_1_max(self,max_plan_dist):
		max_f1 = self.distance_dict["linear"](max_plan_dist)
		max_f2 = self.distance_dict["exponential"](max_plan_dist)
		max_f3 = self.distance_dict["log"](max_plan_dist)
		max_ppl1 = 1;max_ppl2 = 1;max_ppl3 = 1;
		max_obs_f1 = 10
		return np.array([max_f1,max_f2,max_f3,max_ppl1,max_ppl2,max_ppl3,max_obs_f1])

	def icra_featureset_2(self,robot_xy,goal_xy):
		goal_dst = np.linalg.norm(robot_xy- goal_xy)
		goal_f1 = self.distance_dict["linear"](goal_dst)
		goal_f2 = self.distance_dict["exponential"](goal_dst)
		goal_f3 = self.distance_dict["log"](goal_dst)
		ppl_f1 = 0;ppl_f2=0;ppl_f3=0
		if self.people_latest!=None:
			dist =to_person_frame(robot_xy,self.simple_ppl_poses)
			ppl_f1= self.angled_step_feature(dist,0.3,1.5)	
			ppl_f2= self.angled_step_feature(dist,0.8,1)
			ppl_f3= self.angled_step_feature(dist,3.15,0.3)
		obstacle_dist = self.obstacle_dist(robot_xy)
		obs_f1 = 4 - obstacle_dist
		if obs_f1 <0:
			obs_f1=0
		return np.array([goal_f1,goal_f2,goal_f3,ppl_f1,ppl_f2,ppl_f3,obs_f1])/self.normaliser
	def icra_featureset_2_max(self,max_plan_dist):
		max_f1 = self.distance_dict["linear"](max_plan_dist)
		max_f2 = self.distance_dict["exponential"](max_plan_dist)
		max_f3 = self.distance_dict["log"](max_plan_dist)
		max_ppl1 = 1;max_ppl2 = 1;max_ppl3 = 1;
		max_obs_f1 = 4
		return np.array([max_f1,max_f2,max_f3,max_ppl1,max_ppl2,max_ppl3,max_obs_f1])

	def get_cost(self,robot_xy,goal_xy):
		feat = self.featureset(robot_xy,goal_xy)
		if self.ref_path_nn != None:
			c = np.dot(self.weights,feat) + self.augmented_cost(robot_xy,self.ref_path_nn)
			if c <0:
				c=0
			return c
			
		else:
			return np.dot(self.weights,feat)
			
	def augmented_cost(self,robot_xy,ref_path_nn):
		(dist, idx) = ref_path_nn.kneighbors(robot_xy.reshape(1,-1))
		return np.amax([-dist[0,0],-0.5])
	def get_cost2(self,robot_xy,goal_xy):
		return 5
	def gaussian_feature(self,xy,mean=np.array([0,0]),cov=np.array([1,1])):
		return np.sum(np.exp(-1./2.*np.sum(((xy-mean)/cov)**2,axis=1)))

	def angled_step_feature(self,xy,angle,distance):
		d = np.linalg.norm(xy,axis=1)
		angles = np.arctan2(xy[:,1],xy[:,0])
		idx1 = np.where(np.abs(angles)<angle)[0]
		idx2 = np.where(d<distance)[0]
		return len(np.intersect1d(idx1,idx2))
	def set_featureset(self,features):
		if features == "icra1":
			self.normaliser = self.icra_featureset_1_max(self.max_plan_dist)
			self.featureset =self.icra_featureset_1
			self.cov1 = np.array([0.8,0.8]);self.cov2 = np.array([1.2,0.2]);self.cov3 = np.array([0.2,1.2])
			self.mean1 = np.array([0.,0.]);self.mean2 = np.array([1.,0.]);self.mean3 = np.array([-1.,0.]);
		elif features == "icra2":
			self.normaliser = self.icra_featureset_2_max(self.max_plan_dist)
			self.featureset =self.icra_featureset_2
		elif features =="icra3":
			self.normaliser = self.icra_featureset_1_max(self.max_plan_dist)
			self.featureset =self.icra_featureset_1
			self.cov1 = np.array([0.3,0.3]);self.cov2 = np.array([0.3,0.3]);self.cov3 = np.array([0.3,0.3])
			self.mean1 = np.array([0.,0.]);self.mean2 = np.array([.7,0.]);self.mean3 = np.array([-.7,0.]);
		

	def edge_cost(self,c1,c2,xy_1,xy_2):
	    d = np.linalg.norm(xy_1-xy_2)
	    return 0.5*(c1+c2)*d

	def edge_feature(self,f1,f2,xy_1,xy_2):
	    d = np.linalg.norm(xy_1-xy_2)
	    return 0.5*(f1+f2)*d

def to_person_frame(robotPose,personPoses):
	vec = -personPoses[:,:2] + robotPose

	length = np.linalg.norm(vec,axis=1) 
	phi = np.arctan2(vec[:,1],vec[:,0])
	psi = phi - personPoses[:,2]
	x = np.cos(psi)*length
	y = np.sin(psi)*length
	return np.vstack([x,y]).T

def linear(x):
	return x
def one_over(x):
	return 15/(1+x)
def safe_log(x):
	return np.log(1+x)

if __name__=='__main__':
	# test_persons = np.array([[1,2,np.pi],[4,5,np.pi/2]])
	# robot = np.array([0,0])
	# out = to_person_frame(robot,test_persons)
	# ou = gaussian_feature(out)
	# print ou
	angles = np.random.uniform(-np.pi,np.pi,1000)
	tic = time.time()
	for i in angles:
		np.arctan2(i)
	toc = time.time()
	print "NUMPY",toc-tic
	tic = time.time()
	for i in angles:
		atan2(i)
	toc = time.time()
	print "PYTHON",toc-tic

