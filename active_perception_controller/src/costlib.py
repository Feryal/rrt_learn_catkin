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
from helper_functions import pixel_to_point,pixel_to_point_p,points_to_index
from geometry_msgs.msg._PoseStamped import PoseStamped
import math

class Cost_Manager(object):
	def __init__(self,dist_trans,nav_map,features = "icra1",max_plan_dist=30):
		self.distance_transform=dist_trans
		self.nav_map=nav_map
		self.distance_dict = {"linear":linear,"exponential":np.exp,"log":safe_log,
								"one_over":one_over}
		self._costmap_pub = rospy.Publisher("rrt_cost",PointCloud,queue_size=1)
		self._goal_sub = rospy.Subscriber("move_base_simple/goal",
                                                      PoseStamped,
                                                      self.goal_cb,
                                                      queue_size=1)

		self.freecells = [i for i in xrange(0,len(self.nav_map.data)) 
                        if self.nav_map.data[i] == 0]

		self._lock = threading.Lock()
		# self.weights = np.ones(7)
		# self.weights[0] = 3.
		# self.weights[1] = 1.
		# self.weights[2] = 0
		# self.weights[-1]=2
		# self.weights[-2]=1
		# self.weights[-3]=7
		# self.weights[-4]=2
		self.weights = np.ones(7)
		self.weights[0] = 1.1
		self.weights[1] = 1.1
		self.weights[2] = 0.
		self.weights[3]=2.5
		self.weights[4]=5.1
		self.weights[5]=1.2
		self.weights[6]=1.
		self.features = features
		self.max_plan_dist = max_plan_dist
		self.set_featureset(features)
		self.people_sub = rospy.Subscriber("person_poses",
                                                      PersonArray,
                                                      self.ppl_cb,
                                                queue_size=1)
		self.people_latest = None
		self.simple_ppl_poses = None
		self.origin = np.array([self.nav_map.info.origin.position.x,self.nav_map.info.origin.position.y])
		self.res = self.nav_map.info.resolution
		self.ref_path_nn =None


	def initialise_costmap(self):
	    self.costmap =PointCloud()
	    self.costmap.header.frame_id="map"
	    self.costmap.header.stamp = rospy.get_rostime()
	    self.map_points = pixel_to_point_p(self.freecells,self.nav_map)
	    for i in self.map_points:
	        p = Point32()
	        p.x = i[0]
	        p.y = i[1]
	        p.z = 0.7 
	        self.costmap.points.append(p)
	    vals = ChannelFloat32()
	    vals.name="cost"
	    vals.values = [0]*len(self.costmap.points)
	    self.costmap.channels.append(vals)
	def update_costmap(self):
		self._lock.acquire()
		self.featureset = self.icra_featureset_p
		goal_xy = [self._goal.pose.position.x,self._goal.pose.position.y]
		vals = ChannelFloat32()
		vals.name = "cost"
		#t = time.time()
		val = self.get_cost_p(self.map_points,np.array(goal_xy))
		#print "TIME FOR COSTMAP",time.time()-t
		self.costmap_np = np.zeros(len(self.nav_map.data))
		self.costmap_np[self.freecells] = val
		self.costmap.channels[0].values = val
		#self.costmap.chasnnels[0].values[n] = self.cost_manager.obstacle_cost(np.array([i[0],i[1]]))
		self._costmap_pub.publish(self.costmap)
		self.set_featureset(self.features)
		self._lock.release()
	def query_costmap(self,robot_xy):
		idx = points_to_index(robot_xy,self.nav_map)
		costs = self.costmap_np[idx]
		return costs
	def ppl_cb(self,msg):
		self.people_latest = msg
		if len(self.people_latest.persons) ==0:
			self.people_latest = None
		else:
			self.simple_ppl_poses = []
			for i in self.people_latest.persons:
				quat = [i.pose.orientation.x,i.pose.orientation.y,i.pose.orientation.z,i.pose.orientation.w]
				ang = tf.transformations.euler_from_quaternion(quat)[2]
				self.simple_ppl_poses.append(np.array([i.pose.position.x,i.pose.position.y,ang]))
			self.simple_ppl_poses = np.array(self.simple_ppl_poses)
	def goal_cb(self,msg):
		self._goal = msg
		self._goal_xy = np.array([msg.pose.position.x,msg.pose.position.y])
	def obstacle_dist(self,robot_xy):
		dt_idx  = np.array([int((robot_xy[0] - self.origin[0])/self.res),int((-robot_xy[1] + self.origin[1])/self.res)])

		try:
			out = self.distance_transform[dt_idx[1],dt_idx[0]]*self.res
		except IndexError:
			pdb.set_trace()
		return out

	def obstacle_dist_p(self,robot_xy):
		dt_idx  = np.array([((robot_xy[:,0] - self.origin[0])/self.res).astype(int),((-robot_xy[:,1] + self.origin[1])/self.res).astype(int)])
		try:
			out = self.distance_transform[dt_idx[1],dt_idx[0]]*self.res
		except IndexError:
			pdb.set_trace()
		return out
	def goal_cost(self,robot_xy,goal_xy):
		return np.exp(np.linalg.norm(robot_xy- goal_xy))
	def icra_featureset_1(self,robot_xy,goal_xy):
		goal_dst = np.linalg.norm(robot_xy- goal_xy)
		goal_f1 = self.distance_dict["linear"](goal_dst)
		goal_f2 = self.distance_dict["exponential"](goal_dst/7)
		goal_f3 = self.distance_dict["log"](goal_dst)
		ppl_f1 = 0;ppl_f2=0;ppl_f3=0
		if self.people_latest!=None and self.simple_ppl_poses!=None:
			dist =to_person_frame(robot_xy,self.simple_ppl_poses)
			self.angled_step_feature(dist,0,0)
			ppl_f1= self.gaussian_feature(dist,mean = self.mean1,cov = self.cov1)
			ppl_f2= self.gaussian_feature(dist,mean = self.mean2,cov = self.cov2)
			ppl_f3= self.gaussian_feature(dist,mean = self.mean3,cov = self.cov3)
		obstacle_dist = self.obstacle_dist(robot_xy)
		obs_f1 = 1/(0.01+obstacle_dist)
		return np.array([goal_f1,goal_f2,goal_f3,ppl_f1,ppl_f2,ppl_f3,obs_f1])/self.normaliser
	def icra_featureset_p(self,robot_xy,goal_xy):
		goal_dst = np.linalg.norm(robot_xy- goal_xy,axis =1)
		goal_f1 = self.distance_dict["linear"](goal_dst)
		goal_f2 = self.distance_dict["exponential"](goal_dst/7)
		goal_f3 = self.distance_dict["log"](goal_dst)
		ppl_f1 = np.zeros(robot_xy.shape[0]);ppl_f2=np.zeros(robot_xy.shape[0]);ppl_f3=np.zeros(robot_xy.shape[0])
		if self.people_latest!=None and self.simple_ppl_poses!=None:
			dist =to_person_frame_p(robot_xy.T,self.simple_ppl_poses)
			self.angled_step_feature(dist,0,0)
			ppl_f1= self.gaussian_feature_p(dist,mean = self.mean1,cov = self.cov1)
			ppl_f2= self.gaussian_feature_p(dist,mean = self.mean2,cov = self.cov2)
			ppl_f3= self.gaussian_feature_p(dist,mean = self.mean3,cov = self.cov3)
		obstacle_dist = self.obstacle_dist_p(robot_xy)
		obs_f1 = 1/(0.01+obstacle_dist)
		return 1/self.normaliser* np.array([goal_f1,goal_f2,goal_f3,ppl_f1,ppl_f2,ppl_f3,obs_f1]).T
	def icra_featureset_1_max(self,max_plan_dist):
		max_f1 = self.distance_dict["linear"](max_plan_dist)
		max_f2 = self.distance_dict["exponential"](max_plan_dist/7)
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

	def get_cost_p(self,robot_xy,goal_xy):
		feat = self.featureset(robot_xy,goal_xy)
		if self.ref_path_nn != None:
			c = np.dot(feat,self.weights) + self.augmented_cost_p(robot_xy,self.ref_path_nn)
			if len(np.where(c <0)[0])>0:
				c[np.where(c<0)[0]] = 0
			return c
		else:
			return np.dot(feat,self.weights)
			
	def augmented_cost(self,robot_xy,ref_path_nn):
		(dist, idx) = ref_path_nn.kneighbors(robot_xy.reshape(1,-1))
		return np.amax([-dist[0,0],-0.5])
	def augmented_cost_p(self,robot_xy,ref_path_nn):
		(dist, idx) = ref_path_nn.kneighbors(robot_xy)
		comp = -0.5 * np.ones(dist.shape[0])
		return np.amax([-dist[:,0],comp],axis=0)
	def gaussian_feature(self,xy,mean=np.array([0,0]),cov=np.array([1,1])):
		return np.sum(np.exp(-1./2.*np.sum(((xy-mean)/cov)**2,axis=1)))
	def gaussian_feature_p(self,xy,mean=np.array([0,0]),cov=np.array([1,1])):
		return np.sum(np.exp(-1./2.*np.sum(((xy-mean.reshape(1,2,1))/cov.reshape(1,2,1))**2,axis=1)),axis=0)

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
	def path_cost(self,path,goal_xy):
		# gives total cost of a set of points(path). Prototyped version. TODO implement something faster.
		costs = self.query_costmap(path[:,:2])
		d = np.linalg.norm(path[:-1,:2]-path[1:,:2],axis = 1)
		d_costs = costs[:-1] + costs[1:]
		tot_cost = np.sum(d*d_costs)*0.5
		return tot_cost
def path_cost_test(path,goal_xy):
	# gives total cost of a set of points(path). Prototyped version. TODO implement something faster.
	cost = 0
	for n in xrange(1,path.shape[0]):
		if n ==1:
			x1 = path[n-1]
			c1 = np.linalg.norm(x1[:2]-goal_xy)
			x2 = path[n]
			c2 = np.linalg.norm(x2[:2]-goal_xy)
		else:
			x1 = x2
			c1 = c2
			x2 = path[n]
			c2 = np.linalg.norm(x2[:2]-goal_xy)
		d = np.linalg.norm(x1-x2)
		cost+=0.5*(c1+c2)*d
	return cost
def to_person_frame(robotPose,personPoses):
	vec = -personPoses[:,:2] + robotPose
	length = np.linalg.norm(vec,axis=1) 
	phi = np.arctan2(vec[:,1],vec[:,0])
	psi = phi - personPoses[:,2]
	x = np.cos(psi)*length
	y = np.sin(psi)*length
	return np.vstack([x,y]).T
def to_person_frame_p(robotPose,personPoses):
	n = personPoses.shape[0]
	l = robotPose.shape[1]
	path_pose = np.array([robotPose]*personPoses.shape[0])
	vec = -personPoses[:,:2].reshape(n,2,1) + path_pose
	vec = vec.swapaxes(1,2)
	vec = vec.reshape(n*l,vec.shape[2])
	phi = np.arctan2(vec[:,1],vec[:,0]).reshape(n,1,l,order = "C")
	length = np.linalg.norm(vec,axis=1) 
	psi = phi - personPoses[:,2].reshape(n,1,1)
	psi = psi.reshape(n*l)
	x = np.cos(psi)*length
	y = np.sin(psi)*length
	return np.vstack([x,y]).T.reshape(n,l,2).swapaxes(1,2)
def linear(x):
	return x
def one_over(x):
	return 15/(1+x)
def safe_log(x):
	return np.log(1+x)
def test_dst(robot_xy,origin = np.array([0,0])):
	res = 0.3
	dt_idx  = np.array([((robot_xy[:,0] - origin[0])/res).astype(int),((-robot_xy[:,1] + origin[1])/res).astype(int)])
	return dt_idx.T
def gaussian_feature(xy,mean=np.array([0,0]),cov=np.array([1,1])):
	return np.sum(np.exp(-1./2.*np.sum(((xy-mean.reshape(1,2,1))/cov.reshape(1,2,1))**2,axis=1)),axis=0)

if __name__=='__main__':
	test_persons = np.array([[1,2,np.pi],[4,5,np.pi/2],[4,5,np.pi/2],[4,5,np.pi/2]])
	#self.simple_ppl_poses = test_persons
	robot = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0]])
	goal_xy = np.array([20,20])
	goal_dst = np.linalg.norm(robot- goal_xy)
	goal_f1 = linear(goal_dst)


	robot = np.random.uniform(size = (1000,2))
	test_persons = np.random.uniform(size  = (10,3))
	t = time.time()
	out = to_person_frame_path(robot.T,test_persons)
	print time.time()-t
	out3 = test_dst(robot)
	pdb.set_trace()
	#cost = path_cost_test(robot,goal_xy)

	# ou = gaussian_feature(out)
	# print ou

