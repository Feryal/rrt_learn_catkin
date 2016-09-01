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
from std_msgs.msg import Float32MultiArray
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
import posq as posq



class MotionPlanner():
    def __init__(self):        
        self._lock = threading.Lock()
        self._lock.acquire()
        
        self._robot_pose_sub = rospy.Subscriber("amcl_pose",
                                                PoseWithCovarianceStamped,
                                                self.robot_pose_cb,
                                                queue_size=1)
        # TODO: change the particles topic name
        self._goal_sub = rospy.Subscriber("move_base_simple/goal",
                                                      PoseStamped,
                                                      self.goal_recieved,
                                                      queue_size=1)

        self._robot_pose = PoseWithCovariance()
        
        self._rrt_pub = rospy.Publisher("rrt",
                                         Path,
                                         queue_size=1,
                                         latch = True)

        self._path_pub = rospy.Publisher("best_path",
                                         Path,
                                         queue_size=1,
                                         latch = True)
        
        self._entropy_pub = rospy.Publisher("entropy_points",
                                         PointCloud,
                                         queue_size=1,
                                         latch = True)

        self._costmap_pub = rospy.Publisher("rrt_cost",PointCloud,queue_size=1)

        self.samp_point_pub = rospy.Publisher("sampled_point",MarkerArray,queue_size=1)
        self.weights_pub = rospy.Publisher("weights",Float32MultiArray,queue_size=1)
 
        getmap = rospy.ServiceProxy('static_map', GetMap)
        
        srv_available = False
        while not srv_available:
            try:
                rospy.wait_for_service('static_map',2.0)
                srv_available = True
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(e.message)
        
        self._goal = None
        self._navmap = getmap().map
        width = self._navmap.info.width
        height = self._navmap.info.height
        self.origin = np.array([self._navmap.info.origin.position.x,self._navmap.info.origin.position.y])
        self.res = self._navmap.info.resolution
        self._freecells = [i for i in xrange(0,len(self._navmap.data)) 
                           if self._navmap.data[i] == 0]
        
        self._rrt_eta = rospy.get_param("~rrt_eta", 1.1) # Notation from Karaman & Frazolli, 2011
        self.planner = rospy.get_param("~planner", "rrtstar")
        robot_radius = rospy.get_param("~robot_radius", 0.4)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.8)
        self.planning_time = rospy.get_param("~planning_time", 100.)
        self.max_planning_time = rospy.get_param("~max_planning_time", 120.)
        self._robot_radius_px = robot_radius / self._navmap.info.resolution
        sigma_person = rospy.get_param("sigma_person", 0.05)
        self.astar_res = rospy.get_param("~astar_res", 0.5)
        self.planning_featureset = rospy.get_param("~cost_featureset", "icra3")

        print "PLANNING TIME",self.planning_time
        self._planned = False # This is just for testing purposes. Delete me!
        #self.planner = "rrtstar"
        mapdata = np.asarray(self._navmap.data, dtype=np.int8).reshape(height, width)
        logical = np.flipud(mapdata == 0)
        self._distmap = sp.ndimage.distance_transform_edt(logical)

        dt = self._distmap;mp = self._navmap
        self.cost_manager = Cost_Manager(dt,mp,features = self.planning_featureset)
        pkgpath = roslib.packages.get_pkg_dir('active_perception_controller')
        self._plan_srv = rospy.Service('plan', ActivePerceptionPlan, self._plan_srv_cb)
        self.cost_manager.initialise_costmap()
        self._lock.release()

    def write_planner_params(self,directory):
        f = open(directory+"/readme","w")
        f.write("Planner parameters used: \n------ \n \n")
        f.write("Planner type:  "+ self.planner +"\n")
        if self.planner == 'astar':
            f.write("A* resolution:  "+str(self.astar_res)+"\n")
        elif self.planner == "rrtstar":
            f.write("RRT* planning time:  "+str(self.planning_time)+"\n")
            f.write("RRT eta:  "+str(self._rrt_eta)+"\n")
        f.write("Featureset:"+self.planning_featureset+"\n")
        f.close()

    def goal_recieved(self,msg):
        self._goal = msg
        print "GOAL RECIEVED"
        #tick = time.time()
        #self.update_costmap()
        #toc = time.time()
        #print "TIME FOR COSTMAPS", toc-tick

    def robot_pose_cb(self, msg):
        self._robot_pose = msg.pose

    def _plan_srv_cb(self, msg):
        if self._goal!=None:
            #pr = cProfile.Profile()
            #pr.enable()
            # ... do something ...
            #path = self.astar()

            pose_path,array_path = self.plan()
            #pr.disable()
            #s = StringIO.StringIO()
            #sortby = 'cumulative'
            #ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            #ps.print_stats()
            #print s.getvalue()
            res = ActivePerceptionPlanResponse()
            res.path = pose_path
            return res
        else:
            pass

    def plan(self):
        self.cost_manager._goal = self._goal
        self.cost_manager.update_costmap()
        self._lock.acquire()
        weights = Float32MultiArray()
        weights.data = self.cost_manager.weights
        self.weights_pub.publish(weights)
        if self.planner == "rrtstar":
            cached_points = self.make_cached_rrt(self.sample_goal_bias,points_to_cache = 2900 )
            #pr = cProfile.Profile()
            #pr.enable()
            pose_path,array_path = self.plan_cached_rrt(cached_points)
            #pr.disable()
            #s = StringIO.StringIO()
            #sortby = 'cumulative'
            #ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            #ps.print_stats()
            #print s.getvalue()
            #pose_path,array_path = self.posq_rrtstar(self.sample_goal_bias)
        else:
            print "NO PLANNER FOUND"
            return None
        print "Done Planning"
        self._lock.release()
        return pose_path,array_path

    def make_cached_rrt(self,sample_fn,points_to_cache = 4500,bias=0.02):
        """
        CAching the RRT
        """
        print "NOW CACHING RRT ---"
        marker_points = MarkerArray()
        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y,2*np.arccos(self._robot_pose.pose.orientation.w)])

        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        p4dist = np.zeros(4)
        p4dist[:2] = probot[:2];p4dist[2] = np.cos(probot[2]);p4dist[3] = np.sin(probot[2])
        V_xy = [p4dist]

        sampled_points = []
        Dist = [0.0]
        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y,2*np.arccos(self._goal.pose.orientation.w)])
        edge_C = {}
        planning_time=self.planning_time
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        lowest_cost_idx = None
        nbrs.fit(V_xy)
        t1 = time.time()
        planning_done = False
        rrt_iter = 0
        bias = 0.02
        stp = 8
        while not planning_done:
            cached_nbrs ={}
            t2 = time.time()
            #bias/=1.001 # gradually reduce the bias for the target
            """
            Sampling new point
            """
            reached = False
            samp_count = 0
            alternative = True
            while reached ==False:
                    
                prand,g_s = sample_fn(goal_xy,bias = bias)
                p4dist = np.zeros(4)
                p4dist[:2] = prand[:2];p4dist[2] = np.cos(prand[2]);p4dist[3] = np.sin(prand[2])
                #(dist, idx) = nbrs.kneighbors(prand.reshape(1, -1))
                (dist, idx) = nbrs.kneighbors(p4dist.reshape(1, -1))
                pnearest_idx = idx.flatten(1)[0]
                pnearest = V[pnearest_idx]

                """
                Turning new point into reachable point
                """


                stp = 50
                path_new,reached,stp = posq.simulate(pnearest,prand,steps = stp,return_steps=True)
                pnew = path_new[-1]
                if alternative == True:
                    d = prand[:2] - pnearest[:2]
                    ang = np.arctan2(d[1],d[0])

                    add = np.array([self._rrt_eta*np.cos(ang),self._rrt_eta*np.sin(ang),ang])
                    pnew =np.zeros(3)
                    pnew[:2] = pnearest[:2]+add[:2]
                    pnew[2] = ang
                
                #self.publish_local_path(pub_path)
                #pnew = [pnearest[0]+ self._rrt_eta*np.cos(pnearest[2]),pnearest[1]+ self._rrt_eta*np.sin(pnearest[2]),pnearest[2]]
                if reached == True:
                    pnew = prand
                elif reached == False:
                    stp = 400
                    path_new,reached,stp = posq.simulate(pnearest,pnew,steps = stp,return_steps = True,eps=0.1)
                    pnew = path_new[-1]
                """
                Checking if segment is valid and updating graph
                """
                #stp = 40
                
                #stp = 30

            """
            Checking if segment is valid and updating graph
            """
            if self.path_safe(path_new) is True:

                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                p4dist = np.zeros(4)
                p4dist[:2] = pnew[:2];p4dist[2] = np.cos(pnew[2]);p4dist[3] = np.sin(pnew[2])
                #Pnear_idx = nbrs.radius_neighbors(pnew.reshape(1, -1)[:,:2], r, return_distance = False)
                Pnear_idx = nbrs.radius_neighbors(p4dist.reshape(1, -1), r, return_distance = False)
                Pnear_idx = Pnear_idx[0]
                cached_nbrs["prand"] = prand
                cached_nbrs["pnearest_idx"] = pnearest_idx
                cached_nbrs["path_new"] = path_new
                cached_nbrs["pnew"] = pnew
                cached_nbrs["path_new"] = path_new
                cached_nbrs["Pnear_idx"] = Pnear_idx
                cached_nbrs["Pnear_forward"] = []
                cached_nbrs["Pnear_backward"] = []
                cached_nbrs["pnear_pnew"] = []
                cached_nbrs["pnew_pnear"] = []
                for p_idx in Pnear_idx:
                    p = V_xy[p_idx]
                    p_xyz = V[p_idx]
                    #path_forward,reached_forward = posq.simulate(p_xyz,pnew,steps = int(stp))
                    path,reached = posq.simulate(p_xyz,pnew,steps = int(stp))
                    if reached == True:
                        safe = self.path_safe(path)
                        if safe is True:
                            path_info = ({"path":path,"reached":reached,"safe":safe})
                            cached_nbrs["pnear_pnew"].append(path_info)
                            cached_nbrs["Pnear_forward"].append(p_idx)
                    # else: 
                    #     path_forward_safe = None

                    path,reached = posq.simulate(pnew,p_xyz,steps = int(stp)) 
                    if reached == True:
                        safe = self.path_safe(path)
                        if safe == True:
                            path_info = ({"path":path,"reached":reached,"safe":safe})
                            cached_nbrs["pnew_pnear"].append(path_info)
                            cached_nbrs["Pnear_backward"].append(p_idx)
                    # else: 
                    #     path_backward_safe = None
                    #path_info = ({"forward":path_forward,"reached_forward":reached_forward,"safe_forward":path_forward_safe,
                    #"backward":path_backward,"reached_backward":reached_backward,"safe_backward":path_backward_safe})
                    #cached_nbrs["pnear_pnew"].append(path_info)
                V.append(pnew)
                V_xy.append(p4dist)
                nbrs.fit(V_xy)
                #mark = self.make_sample_marker(pnew)
                #marker_points.markers.append(mark)
                sampled_points.append(cached_nbrs)
            rrt_iter +=1
            if len(V) == points_to_cache-50:
                bias = 0.9
            if len(V) == points_to_cache:
                planning_done=True

                print "Number of cached points:",len(V)
                print "time taken",time.time()-t1
        return sampled_points

    def plan_cached_rrt(self,cached_points):

        """
        Plan cached RRT
        """
        print "Planning a cached RRT*"
        marker_points = MarkerArray()
        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        print "FREE CELL VOLUME", vol_freecells
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)

        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y,2*np.arccos(self._robot_pose.pose.orientation.w)])
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        #V_xy = [probot[:2]]
        p4dist = np.zeros(4)
        p4dist[:2] = probot[:2];p4dist[2] = np.cos(probot[2]);p4dist[3] = np.sin(probot[2])
        V_xy = [p4dist]
        E = {}
        parents = {}
        Dist = [0.0]

        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y,2*np.arccos(self._goal.pose.orientation.w)])
        c_init = self.cost_manager.get_cost(probot[:2],goal_xy[:2])
        edge_C = {}
        #flann = FLANN()
        lowest_cost_idx = None
        #params = flann.build_index(np.array(V))
        #pdb.set_trace()
        t1 = time.time()

        planning_done = False
        rrt_iter = 0
        while not planning_done:
            t2 = time.time()
            """
            Sampling new point
            """
            cached = cached_points[rrt_iter]
            prand = cached["prand"]
            pnearest_idx = cached["pnearest_idx"]
            pnearest = V[pnearest_idx]
            """
            Turning new point into reachable point
            """
            pnew =cached["pnew"]
            path_new = cached["path_new"]
            Pnear_idx = cached["Pnear_idx"]
            pmin_idx = pnearest_idx

            cum_c = self.integrate_costs(edge_C,parents,pnearest_idx)
            min_edge_c = self.cost_manager.path_cost(path_new,goal_xy[:2])
            cmin = cum_c +min_edge_c
            cumulative_costs = {}
            Pnear_fwd = cached["Pnear_forward"]
            for num,p_idx in enumerate(Pnear_fwd):
                p = V_xy[p_idx]
                p_xyz = V[p_idx]
                cum_cost = self.integrate_costs(edge_C,parents,p_idx)
                cumulative_costs[p_idx] = cum_cost
                p_idx_path = cached["pnear_pnew"][num]["path"]
                reached = cached["pnear_pnew"][num]["reached"]
                safe = cached["pnear_pnew"][num]["safe"]
                if reached is True and safe is True:
                    path_c = self.cost_manager.path_cost(p_idx_path,goal_xy[:2])
                else:
                    path_c = 0
                #reached = False
                c = cum_cost + path_c
                if (safe is True and
                    reached is True and c < cmin):
                    cmin = c
                    min_edge_c = path_c
                    pmin_idx = p_idx      

            if E.has_key(pmin_idx):
                E[pmin_idx].add(len(V))
            else:
                E[pmin_idx] = set([len(V)])   
            edge_C[pmin_idx,len(V)] = min_edge_c  
            cumulative_last = cmin     
            pnew_idx = len(V)
            V.append(pnew)
            p4dist = np.zeros(4)
            p4dist[:2] = pnew[:2];p4dist[2] = np.cos(pnew[2]);p4dist[3] = np.sin(pnew[2])
            V_xy.append(p4dist)
            parents[pnew_idx] = pmin_idx
            """
            Re-wire the tree
            """
            unsafe = 0
            Pnear_bwd = cached["Pnear_backward"]
            for en,p_idx in enumerate(Pnear_bwd):
                if parents.has_key(p_idx):
                    if not cumulative_costs.has_key(p_idx):
                        cumulative_costs[p_idx] = self.integrate_costs(edge_C,parents,p_idx)
                    p_xyz = V[p_idx]
                    rewire_path = cached["pnew_pnear"][en]["path"]
                    rewire_reached = cached["pnew_pnear"][en]["reached"]
                    rewire_safe = cached["pnew_pnear"][en]["safe"]
                    #rewire_path,rewire_reached = posq.simulate(pnew,p_xyz,steps = int(stp))
                    if rewire_reached is True and rewire_safe is True :
                        rewire_path_c = self.cost_manager.path_cost(rewire_path,goal_xy[:2])
                    else:
                        rewire_path_c = 0
                    c = cumulative_last + rewire_path_c
                    if (rewire_safe is True and c < cumulative_costs[p_idx] and rewire_reached is True):
                        E[parents[p_idx]].remove(p_idx)
                        edge_C.pop(parents[p_idx],p_idx)
                        edge_C[pnew_idx,p_idx] = rewire_path_c
                        parents[p_idx] = pnew_idx
                        if E.has_key(pnew_idx):
                            E[pnew_idx].add(p_idx)
                        else:
                            E[pnew_idx] = set([p_idx])
            rrt_iter +=1
            if rrt_iter==len(cached_points):
                planning_done=True
                nbrs.fit(V_xy)
                p4dist = np.zeros(4)
                p4dist[:2] = goal_xy[:2];p4dist[2] = np.cos(goal_xy[2]);p4dist[3] = np.sin(goal_xy[2])
                points_near_goal = []
                add = 0
                while len(points_near_goal)==0:
                    dist,points_near_goal = nbrs.radius_neighbors(p4dist, self.goal_tolerance+add, return_distance = True)
                    points_near_goal = points_near_goal[0]
                    add +=0.1
                print "DONE PLANNING"
                print "TIME TAKEN",time.time()-t1
                print "POINTS NEAR GOAL",points_near_goal

        #self.samp_point_pub.publish(marker_points)
        """
        Find best path:
        """
        min_cost = None;
        for i in points_near_goal:
            c_path = self.integrate_costs(edge_C,parents,i)
            if c_path < min_cost or min_cost==None:
                m = i
                min_cost = c_path
        self.publish_rrt(V,E)   
        print "MINIMUM PATH COST RRT",min_cost
        path = self.get_path(parents,V,m)
        pt = path_to_pose(path)            
        print 'total time: ', time.time()-t1 
        self._path_pub.publish(pt)
        return pt,path

    
    def posq_rrtstar(self, sample_fn,bias = 0.06):
        """
        RRT* Algorithm
        """
        print "KINODYNAMIC PLANNING"
        marker_points = MarkerArray()

        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        print "FREE CELL VOLUME", vol_freecells
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y,2*np.arccos(self._robot_pose.pose.orientation.w)])

        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        p4dist = np.zeros(4)
        p4dist[:2] = probot[:2];p4dist[2] = np.cos(probot[2]);p4dist[3] = np.sin(probot[2])
        V_xy = [p4dist] # V_xy is the data structure for the nearest neighbours
        #V_xy = [probot[:2]]
        E = {}
        parents = {}
        Dist = [0.0]
        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y,2*np.arccos(self._goal.pose.orientation.w)])
        c_init = self.cost_manager.get_cost(probot[:2],goal_xy[:2])
        edge_C = {}
        planning_time=self.planning_time
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        lowest_cost_idx = None
        nbrs.fit(V_xy)
        t1 = time.time()
        planning_done = False
        rrt_iter = 0
        pub_path = []

        while not planning_done:
            t2 = time.time()
            #bias*=1.001 # gradually reduce the bias for the target
            """
            Sampling new point
            """
            reached = False
            samp_count = 0
            alternative = True
            while reached ==False:
                    
                prand,g_s = sample_fn(goal_xy,bias = bias)
                p4dist = np.zeros(4)
                p4dist[:2] = prand[:2];p4dist[2] = np.cos(prand[2]);p4dist[3] = np.sin(prand[2])
                #(dist, idx) = nbrs.kneighbors(prand.reshape(1, -1))
                (dist, idx) = nbrs.kneighbors(p4dist.reshape(1, -1))
                pnearest_idx = idx.flatten(1)[0]
                pnearest = V[pnearest_idx]

                """
                Turning new point into reachable point
                """
                stp = 50
                path_new,reached,stp = posq.simulate(pnearest,prand,steps = stp,return_steps=True)
                pnew = path_new[-1]
                if alternative == True:
                    d = prand[:2] - pnearest[:2]
                    ang = np.arctan2(d[1],d[0])

                    add = np.array([self._rrt_eta*np.cos(ang),self._rrt_eta*np.sin(ang),ang])
                    pnew =np.zeros(3)
                    pnew[:2] = pnearest[:2]+add[:2]
                    pnew[2] = ang                
                #self.publish_local_path(pub_path)
                #pnew = [pnearest[0]+ self._rrt_eta*np.cos(pnearest[2]),pnearest[1]+ self._rrt_eta*np.sin(pnearest[2]),pnearest[2]]
                if reached == True:
                    pnew = prand
                elif reached == False:
                    stp = 400
                    path_new,reached,stp = posq.simulate(pnearest,pnew,steps = stp,return_steps = True,eps=0.1)
                    pnew = path_new[-1]
                """
                Checking if segment is valid and updating graph
                """
                #stp = 40
                
                #stp = 30

            if self.path_safe(path_new):
                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                p4dist = np.zeros(4)
                p4dist[:2] = pnew[:2];p4dist[2] = np.cos(pnew[2]);p4dist[3] = np.sin(pnew[2])
                Pnear_idx = nbrs.radius_neighbors(p4dist.reshape(1, -1), r, return_distance = False)
                #Pnear_idx = nbrs.radius_neighbors(p4dist.reshape(1, -1), r, return_distance = False)
                Pnear_idx = Pnear_idx[0]
                pmin_idx = pnearest_idx
                min_edge_c = self.cost_manager.path_cost(path_new,goal_xy[:2])
                cum_c = self.integrate_costs(edge_C,parents,pnearest_idx)
                cmin = cum_c +min_edge_c
                #if len(Pnear_idx)>5:
                #   Pnear_idx = Pnear_idx[:5]
                cumulative_costs = []
                for p_idx in Pnear_idx:
                    p = V_xy[p_idx]
                    p_xyz = V[p_idx]

                    cum_cost = self.integrate_costs(edge_C,parents,p_idx)
                    cumulative_costs.append(cum_cost)
                    # WATCH OUT. You might get a nearest neightbour problem if the steps are not good enough.
                    # perhaps we can have a distance simulation so that the nearest neighbor calculation remains consistent.
                    p_idx_path,reached = posq.simulate(p_xyz,pnew,steps = int(stp),eps = 0.1)
                    #reached = False
                    safe = self.path_safe(p_idx_path)
                    if reached == True and safe == True:
                        path_c = self.cost_manager.path_cost(p_idx_path,goal_xy[:2])
                    else:
                        path_c = 0
                    #reached = False
                    c = cum_cost + path_c
                    if (safe is True and
                        reached is True and c < cmin):
                        cmin = c
                        min_edge_c = path_c
                        pmin_idx = p_idx      

                if E.has_key(pmin_idx):
                    E[pmin_idx].add(len(V))
                else:
                    E[pmin_idx] = set([len(V)])   
                edge_C[pmin_idx,len(V)] = min_edge_c  
                cumulative_last = cmin     
                pnew_idx = len(V)
                V.append(pnew)
                #V_xy.append(pnew[:2])
                V_xy.append(p4dist)
                parents[pnew_idx] = pmin_idx
                """
                Re-wire the tree
                """
                for en,p_idx in enumerate(Pnear_idx):
                    # so if the near nodes, have children
                    #parent
                    if parents.has_key(p_idx):
                        p = V_xy[p_idx]
                        p_xyz = V[p_idx]
                        rewire_path,rewire_reached = posq.simulate(pnew,p_xyz,steps = int(stp),eps = 0.1)
                        #rewire_reached = False
                        rewire_safe = self.path_safe(rewire_path)
                        if rewire_reached == True and rewire_safe == True:
                            rewire_path_c = self.cost_manager.path_cost(rewire_path,goal_xy[:2])
                        else:
                            rewire_path_c = 0
                        c = cumulative_last + rewire_path_c

                        if (rewire_safe is True and c < cumulative_costs[en] and rewire_reached is True):
                            E[parents[p_idx]].remove(p_idx)
                            edge_C.pop(parents[p_idx],p_idx)
                            edge_C[pnew_idx,p_idx] = rewire_path_c
                            parents[p_idx] = pnew_idx
                            if E.has_key(pnew_idx):
                                E[pnew_idx].add(p_idx)
                            else:
                                E[pnew_idx] = set([p_idx])
                nbrs.fit(V_xy)

            rrt_iter +=1

            if time.time()-t1>self.max_planning_time:
                p4dist = np.zeros(4)
                p4dist[:2] = goal_xy[:2];p4dist[2] = np.cos(goal_xy[2]);p4dist[3] = np.sin(goal_xy[2])
                dist,points_near_goal = nbrs.radius_neighbors(p4dist, self.goal_tolerance+0.2, return_distance = True)
                points_near_goal = points_near_goal[0]
                points_near_goal = []
                add = 0
                while len(points_near_goal)==0:
                    dist,points_near_goal = nbrs.radius_neighbors(p4dist, self.goal_tolerance+add, return_distance = True)
                    points_near_goal = points_near_goal[0]
                    add +=0.1
                print "Could not find solution for 10 seconds, going with solution closest to goal."
                planning_done = True
            elif time.time()-t1>planning_time:
                p4dist = np.zeros(4)
                p4dist[:2] = goal_xy[:2];p4dist[2] = np.cos(goal_xy[2]);p4dist[3] = np.sin(goal_xy[2])
                dist,points_near_goal = nbrs.radius_neighbors(p4dist, self.goal_tolerance+0.2, return_distance = True)
                #dist,points_near_goal = nbrs.radius_neighbors(goal_xy, self.goal_tolerance, return_distance = True)
                dist,point = nbrs.kneighbors(p4dist)   
                print "DISTANCE FROM CLOSEST",dist
                points_near_goal = points_near_goal[0]
                if len(points_near_goal)==0:
                    planning_done = False
                    planning_time+=5.
                    if bias < 0.5:
                        bias =0.9
                else:
                    planning_done = True
            #self.publish_rrt(V,E)     

        #self.samp_point_pub.publish(marker_points)
        """
        Find best path:
        """
        min_cost = 20000000;
        for i in points_near_goal:
            c_path = self.integrate_costs(edge_C,parents,i)
            if c_path < min_cost:
                m = i
                min_cost = c_path
        print len(V)
        self.publish_rrt(V,E)   
        print "MINIMUM PATH COST RRT",min_cost
        path = self.get_path(parents,V,m)
        pt = path_to_pose(path)            
        print 'total time: ', time.time()-t1 
        self._path_pub.publish(pt)
        return pt,path


    def integrate_costs(self,costmap,parents,index):
        # PARENT OF KEY IS VALUE
        at_root = False
        if index==0:
            at_root=True
        cum_cost = 0
        while at_root == False:
            parent = parents[index]
            cum_cost+=costmap[parent,index]
            index = parent
            if parent ==0:
                at_root=True
        return cum_cost

    def publish_rrt(self, V,E):
        pt = Path()
        pt.header.frame_id = '/map'
        path = np.array([V[0]])
        prev_path = np.array([V[0]])
        vis = set()
        path = self.gen_path(0, 0, V, E, path,vis,prev_path)
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pt.poses.append(pose)
            last_pose = pose
        self._rrt_pub.publish(pt)

    def publish_local_path(self,path):
        pt = Path()
        pt.header.frame_id = "/map"
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pt.poses.append(pose)
            last_pose = pose
        self._rrt_pub.publish(pt)


    def smoothing(self,points):
        # gaussian filter for each dimention
        points = np.array(points)
        x_smooth = gaussian_filter(points[:,0],1,mode="nearest")
        y_smooth = gaussian_filter(points[:,1],1,mode="nearest")
        return np.vstack([x_smooth,y_smooth]).T


    def get_path(self, parents, V,idx):
        m=idx
        path = np.array([V[m]])
        at_root = False
        c = 0
        while not at_root and c < len(V):
            p = V[m]
            if m == 0:
                at_root = True
            else:
                m = parents[m]
            path_new,reached = posq.simulate(V[m],p,steps=4000,eps=0.1)
            path = np.append(path,np.flipud(path_new),axis=0)
            c += 1
        if not at_root:
            rospy.logerr("Could not find RRT root! Exiting at node %d",m)
        return path 

    def get_path_points(self, parents, V,idx):
        m=idx
        path = []
        at_root = False
        c = 0
        while not at_root and c < len(V):
            p = V[m]
            path.append(p)            
            if m == 0:
                at_root = True
                print "AT ROOT IF"
            else:
                m = parents[m]
            c += 1
        if not at_root:
            rospy.logerr("Could not find RRT root! Exiting at node %d",m)
        return path          

    def make_sample_marker(self,sampled_point):
        mark = Marker()
        mark.header.frame_id = 'map'
        mark.header.stamp = rospy.get_rostime()
        mark.type = Marker.CUBE
        mark.pose.position.z = 1.
        mark.pose.position.x = sampled_point[0]
        mark.pose.position.y = sampled_point[1]
        mark.scale.x = 0.2;mark.scale.y=0.2;mark.scale.z=0.2
        mark.color.a=1.0;mark.color.b=1.0
        return mark

        
    def gen_path(self, ix, p_ix, V, E, path,vis, path_segment ):
        path = np.append(path,path_segment,axis = 0)
        vis.add(ix)
        if E.has_key(ix):
            for c in E[ix]:
                if c not in vis:
                    path_seg,reached = posq.simulate(V[ix],V[c],steps = 2000,eps=0.1)
                    path =self.gen_path(c, ix, V, E, path,vis, path_seg)    
        path = np.append(path,np.flipud(path_segment),axis = 0)
        return path
    def segment_safe(self, org, dst):
        org_idx = self.point_to_pixels(np.array(org))
        dst_idx = self.point_to_pixels(np.array(dst))
        v1 = self.distance_transform_search(org_idx, dst_idx)
        return v1
    def point_to_pixels(self,point):
        idx = np.array([int((point[0] - self.origin[0])/self.res),int((-point[1] + self.origin[1])/self.res)])
        return idx

    def path_safe(self,path):
        idx = self.vector_to_pixels(path)
        if np.sum(self._distmap[idx[:,1],idx[:,0]]<self._robot_radius_px) == 0:
            return True
        else:
            return False

    def vector_to_pixels(self,path):
        idx = np.asarray([(path[:,0] - self.origin[0])/self.res,(-path[:,1] + self.origin[1])/self.res],dtype="int").T
        return idx
        

    def point_valid(self,point):
        org_idx = self.point_to_pixels(point)
        if self._distmap[org_idx[1],org_idx[0]]==0:
            return False
        else:
            return True

    def distance_transform_search(self, org_idx, dst_idx):
        if self._distmap[dst_idx[1],dst_idx[0]] < self._robot_radius_px:
            return False
        
        alpha = atan2(dst_idx[1]-org_idx[1],
                      dst_idx[0]-org_idx[0])
        ca = cos(alpha)
        sa = sin(alpha)
        ridx = org_idx

        end =False
        while not (ridx[0] == dst_idx[0] and ridx[1] == dst_idx[1]) :
            dist = self._distmap[int(ridx[1]),
                                 int(ridx[0])]
            if dist < self._robot_radius_px:
                return False

            elif np.linalg.norm(ridx - dst_idx) < dist:
                return True
            else:
                ridx = ridx + np.array([ca, sa])*dist
        return True
    
    def sample_free_uniform(self):
        p = np.zeros(3)
        r = np.random.randint(0,len(self._freecells),1)
        idx = self._freecells[r[0]]
        p[:2] = pixel_to_point(idx,self._navmap) 
        p[2] = np.random.uniform(-np.pi,np.pi)
        return p
    def sample_free_uniform_alt(self):
        r = np.random.randint(0,len(self._freecells),1)
        idx = self._freecells[r[0]]
        return pixel_to_point(idx,self._navmap)

    def sample_informed(self,importance_points,variance=1,bias=0.8):
        point = [0,0]
        if np.random.binomial(1,bias)==0:
            point =  self.sample_free_uniform()
        else:
            r = np.random.randint(0,len(importance_points),1)
            valid = False
            while valid ==False:
                noise = np.random.uniform(-variance,variance,size=2)
                point = importance_points[r]+noise
                valid = self.point_valid(point)
        return point
    def sample_goal_bias(self,goal_xy,bias = 0.001):
        # sample a gaussian around the goal with probability bias
        goal_sample = False
        point = np.zeros(3)
        if np.random.binomial(1,bias)==0:
            point =  self.sample_free_uniform()
        else:
            noise = np.random.normal(size=2,scale= 0.2)
            point[:2] = goal_xy[:2]+noise
            point[2] = goal_xy[2]
            goal_sample = True
        return point,goal_sample
    def publish_empty_path(self):
        p = Path()
        p1 = PoseStamped()
        p.header.stamp = rospy.get_rostime()
        p.header.frame_id = "map"
        p.poses.append(p1)
        self._path_pub.publish(p)
        p = Path()
        p1 = PoseStamped()
        p.header.stamp = rospy.get_rostime()
        p.header.frame_id = "map"
        p.poses.append(p1)
        self._rrt_pub.publish(p)

def path_to_pose(path):
    pt = Path()
    pt.header.frame_id = '/map'
    for p in path:
        pose = PoseStamped()
        pose.header.frame_id = '/map'
        # pose.header.seq s= c
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0 #sin(th/2.0)
        pose.pose.orientation.w = 1 #cos(th/2.0)
        pt.poses.append(pose)
    pt.poses.reverse() # fundamental, since poses were added from the end to the beginning
    return pt
    
if __name__=='__main__':
    rospy.init_node('posq_rrt_planner')
    m = MotionPlanner()
    rospy.spin()
