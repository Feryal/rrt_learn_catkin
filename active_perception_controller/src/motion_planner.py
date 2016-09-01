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
from helper_functions import pixel_to_point,pixel_to_point_p,points_to_index
import cProfile, pstats, StringIO
from scipy.ndimage.filters import gaussian_filter
import Queue as Q
from copy import deepcopy



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
        self.samp_point_pub = rospy.Publisher("sampled_point",MarkerArray,queue_size=1)
        self.weights_pub = rospy.Publisher("weights",Float32MultiArray,queue_size=1)
 
        getmap = rospy.ServiceProxy('static_map', GetMap)
        
        srv_available = False
        rospy.sleep(0.4)
        while not srv_available:
            try:
                rospy.wait_for_service('static_map',2.0)
                srv_available = True
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(e.message)

        self._navmap = getmap().map
        self._goal = None
        maps_separate = False

        width = self._navmap.info.width
        height = self._navmap.info.height
        self.origin = np.array([self._navmap.info.origin.position.x,self._navmap.info.origin.position.y])
        self.res = self._navmap.info.resolution
        self._freecells = [i for i in xrange(0,len(self._navmap.data)) 
                           if self._navmap.data[i] == 0]
        
        self._rrt_eta = rospy.get_param("~rrt_eta", 2.0) # Notation from Karaman & Frazolli, 2011
        self.planner = rospy.get_param("~planner", "rrtstar")
        print "THE PLANNER IIIIISSSS", self.planner
        print "RRT ETA",self._rrt_eta
        robot_radius = rospy.get_param("~robot_radius", 0.4)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.2)
        self.planning_time = rospy.get_param("~planning_time", 10.)
        self.max_planning_time = rospy.get_param("~max_planning_time", 20.)
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
        self.cost_manager.update_costmap()
        self._lock.acquire()
        print "PLANNING"
        print self.planner
        weights = Float32MultiArray()
        weights.data = self.cost_manager.weights
        self.weights_pub.publish(weights)
        if self.planner == "rrtstar":
            #cached_points = self.make_cached_rrt(self.sample_goal_bias )
            #pr = cProfile.Profile()
            #pr.enable()
            #pose_path,array_path = self.plan_cached_rrt(cached_points)
            #pr.disable()
            #s = StringIO.StringIO()
            #sortby = 'cumulative'
            #ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            #ps.print_stats()
            #print s.getvalue()
            pose_path,array_path = self.rrtstar(self.sample_goal_bias)
        elif self.planner == "astar":
            pose_path,array_path = self.astar()
        else:
            print "NO PLANNER FOUND"
            return None
        print "Done Planning"
        self._lock.release()
        return pose_path,array_path
    
    def make_cached_rrt(self,sample_fn,points_to_cache = 3500,bias=0.3):
        """
        RRT* Algorithm
        """
        marker_points = MarkerArray()
        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])

        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        sampled_points = []
        Dist = [0.0]
        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y])
        edge_C = {}
        planning_time=self.planning_time
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        lowest_cost_idx = None
        nbrs.fit(V)
        t1 = time.time()
        planning_done = False
        rrt_iter = 0
        bias = 0.1
        while not planning_done:
            cached_nbrs ={}
            t2 = time.time()
            #bias*=1.0025 # increase the goal bias as the RRT progresses
            """
            Sampling new point
            """
            prand = sample_fn(goal_xy,bias = bias)   
            (dist, idx) = nbrs.kneighbors(prand.reshape(1, -1))
            pnearest_idx = idx.flatten(1)[0]
            pnearest = V[pnearest_idx]
            """
            Turning new point into reachable point
            """
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(pnearest, prand)
            """
            Checking if segment is valid and updating graph
            """
            if self.segment_safe(V[pnearest_idx],pnew) is True:

                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                Pnear_idx = nbrs.radius_neighbors(pnew.reshape(1, -1), r, return_distance = False)
                #Pnear_idx,pnear_dist = flann.nn_radius(pnew, r)
                Pnear_idx = Pnear_idx[0]
                cached_nbrs["prand"] = prand
                cached_nbrs["pnearest_idx"] = pnearest_idx
                cached_nbrs["pnew"] = pnew
                cached_nbrs["Pnear_idx"] = Pnear_idx
                V.append(pnew)
                nbrs.fit(V)
                #mark = self.make_sample_marker(pnew)
                #marker_points.markers.append(mark)
                #flann.build_index(np.array(V))
                sampled_points.append(cached_nbrs)
            rrt_iter +=1


            if len(V) == points_to_cache:
                planning_done=True

                print "Number of cached points:",len(V)
                print "time taken",time.time()-t1
        return sampled_points



    def rrtstar(self, sample_fn,bias = 0.1):
        """
        RRT* Algorithm
        """
        marker_points = MarkerArray()

        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        print "FREE CELL VOLUME", vol_freecells
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])

        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        E = {}
        parents = {}
        Dist = [0.0]
        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y])
        c_init = self.cost_manager.get_cost(probot,goal_xy)
        #c_init = self.cost_manager.query_costmap(probot)
        C = [c_init]
        edge_C = {}
        planning_time=self.planning_time
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        lowest_cost_idx = None
        nbrs.fit(V)
        t1 = time.time()
        sampling_informed = False
        planning_done = False
        rewire_counter = 0
        rewire_limit = 55500
        rrt_iter = 0
        importance_points=None

        while not planning_done:
            t2 = time.time()
            #bias*=1.0025 # increase the goal bias as the RRT progresses
            if bias>=1:
                bias=1.
            """
            Sampling new point
            """
            if True:
            #if True:
                prand = sample_fn(goal_xy,bias = bias)   
            else:
                prand = self.sample_informed(importance_points,variance=6) 
            (dist, idx) = nbrs.kneighbors(prand.reshape(1, -1))
            pnearest_idx = idx.flatten(1)[0]
            pnearest = V[pnearest_idx]
            #mrk = self.make_sample_marker(prand)
            #marker_points.markers.append(mrk)
            #print len(marker_points.markers)
            #self.samp_point_pub.publish(mrk)

            """
            Turning new point into reachable point
            """
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(pnearest, prand)
            """
            Checking if segment is valid and updating graph
            """
            if self.segment_safe(V[pnearest_idx],pnew) is True:

                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                Pnear_idx = nbrs.radius_neighbors(pnew.reshape(1, -1), r, return_distance = False)
                #Pnear_idx,pnear_dist = flann.nn_radius(pnew, r)
                Pnear_idx = Pnear_idx[0]
                pmin_idx = pnearest_idx
                c_nearest = self.cost_manager.get_cost(V[pnearest_idx],goal_xy)
                #c_nearest = self.cost_manager.query_costmap(V[pnearest_idx])[0]
                c_new =self.cost_manager.get_cost(pnew,goal_xy)
                #c_new =self.cost_manager.query_costmap(pnew)[0]
                cum_c = self.integrate_costs(edge_C,parents,pnearest_idx)
                min_edge_c =self.cost_manager.edge_cost(c_nearest,c_new,pnearest,pnew)
                cmin = cum_c +min_edge_c
                #if len(Pnear_idx)>5:
                #   Pnear_idx = Pnear_idx[:5]
                cumulative_costs = []
                for p_idx in Pnear_idx:
                    p = V[p_idx]
                    c_near = C[p_idx]

                    cum_cost = self.integrate_costs(edge_C,parents,p_idx)
                    cumulative_costs.append(cum_cost)
                    edge_c = self.cost_manager.edge_cost(c_near,c_new,p,pnew)
                    c = cum_cost + edge_c

                    if (self.segment_safe(p,pnew) is True and 
                        c < cmin):
                        cmin = c
                        min_edge_c = edge_c
                        pmin_idx = p_idx      

                if E.has_key(pmin_idx):
                    E[pmin_idx].add(len(V))
                else:
                    E[pmin_idx] = set([len(V)])   
                edge_C[pmin_idx,len(V)] = min_edge_c  
                cumulative_last = cmin     
                pnew_idx = len(V)
                V.append(pnew)
                C.append(c_new)
                parents[pnew_idx] = pmin_idx
                """
                Re-wire the tree
                """
                for en,p_idx in enumerate(Pnear_idx):
                    # so if the near nodes, have children
                    #parent
                    if parents.has_key(p_idx):
                        p = V[p_idx]
                        c_near = C[p_idx]
                        e_c = self.cost_manager.edge_cost(c_near,c_new,p,pnew)
                        c = cumulative_last + e_c
                        if (self.segment_safe(p,pnew) is True and 
                            c < cumulative_costs[en]):
                            E[parents[p_idx]].remove(p_idx)
                            edge_C.pop(parents[p_idx],p_idx)
                            edge_C[pnew_idx,p_idx] = e_c
                            parents[p_idx] = pnew_idx
                            rewire_counter+=1
                            if E.has_key(pnew_idx):
                                E[pnew_idx].add(p_idx)
                            else:
                                E[pnew_idx] = set([p_idx])
                nbrs.fit(V)

                # Importance point sampling managing.
                pnew_dist = np.linalg.norm(pnew - goal_xy)
                if sampling_informed == False and pnew_dist<self.goal_tolerance:
                    lowest_cost_idx = len(V)-1
                    sampling_informed=True
                    print "SAMPLING IS NOW INFORMED"
                    importance_points = self.get_path( parents, V,lowest_cost_idx)
                elif sampling_informed == True and pnew_dist<self.goal_tolerance:
                    if C[lowest_cost_idx]>C[-1]:
                        lowest_cost_idx = len(V)-1

                if sampling_informed==True and rewire_counter>=rewire_limit:
                    #importance_points.append(pnew)
                    importance_points = self.get_path( parents, V,lowest_cost_idx)
                    rewire_counter=0
                    print "rewire_counter_reset"
                #flann.build_index(np.array(V))
            rrt_iter +=1
            if time.time()-t1>planning_time:
                dist,points_near_goal = nbrs.radius_neighbors(goal_xy, self.goal_tolerance, return_distance = True)
                points_near_goal = points_near_goal[0]
                if len(points_near_goal)==0:
                    planning_done = False
                    planning_time+=0.3
                else:
                    planning_done = True
            if time.time()-t1>self.max_planning_time and planning_done!=True:
                dist,point = nbrs.kneighbors(goal_xy)
                points_near_goal=point[0]
                planning_done=True
                print "Could not find solution for 10 seconds, going with solution closest to goal."


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
        print "MINIMUM PATH COST RRT",min_cost
        path = self.get_path(parents,V,m)
        #path = self.smoothing(path)
        pt = path_to_pose(path)            
        print 'total time: ', time.time()-t1
        
        self.publish_rrt(V,E)        
        self._path_pub.publish(pt)
        print "LENGTH OF", len(C)
        return pt,path

    def plan_cached_rrt(self,cached_points):
        """
        RRT* Algorithm
        """
        marker_points = MarkerArray()

        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        print "FREE CELL VOLUME", vol_freecells
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="kd_tree",leaf_size = 30)
        # V is a list of edges. E is a disctionary where the key is connected with its values.
        # parents is  a disctionary where parent of key is value. Since is a key, each node has only one parent
        V = [probot]
        E = {}
        parents = {}
        Dist = [0.0]

        # C stores the cost at vertex idx which is hte sum of the edges going to it.
        goal_xy = np.array([self._goal.pose.position.x,self._goal.pose.position.y])
        c_init = self.cost_manager.get_cost(probot,goal_xy)
        edge_C = {}
        C = [c_init]
        #flann = FLANN()
        lowest_cost_idx = None
        #params = flann.build_index(np.array(V))
        #pdb.set_trace()
        t1 = time.time()

        planning_done = False
        rrt_iter = 0
        while not planning_done:
            t2 = time.time()
            #bias*=1.0025 # increase the goal bias as the RRT progresses
            """
            Sampling new point
            """
            cached = cached_points[rrt_iter]
            prand = cached["prand"]
            pnearest_idx = cached["pnearest_idx"]
            pnearest = V[pnearest_idx]
            #mrk = self.make_sample_marker(prand)
            #marker_points.markers.append(mrk)
            #print len(marker_points.markers)
            #self.samp_point_pub.publish(mrk)

            """
            Turning new point into reachable point
            """
            pnew =cached["pnew"]
            #Pnear_idx,pnear_dist = flann.nn_radius(pnew, r)
            Pnear_idx = cached["Pnear_idx"]
            pmin_idx = pnearest_idx
            c_nearest = C[pnearest_idx]

            c_new =self.cost_manager.get_cost(pnew,goal_xy)
            cum_c = self.integrate_costs(edge_C,parents,pnearest_idx)
            min_edge_c =self.cost_manager.edge_cost(c_nearest,c_new,pnearest,pnew)
            cmin = cum_c +min_edge_c
            #if len(Pnear_idx)>5:
            #   Pnear_idx = Pnear_idx[:5]
            cumulative_costs = []
            for p_idx in Pnear_idx:
                p = V[p_idx]
                c_near = C[p_idx]

                cum_cost = self.integrate_costs(edge_C,parents,p_idx)
                cumulative_costs.append(cum_cost)
                edge_c = self.cost_manager.edge_cost(c_near,c_new,p,pnew)
                c = cum_cost + edge_c

                if (self.segment_safe(p,pnew) is True and 
                    c < cmin):
                    cmin = c
                    min_edge_c = edge_c
                    pmin_idx = p_idx      

            if E.has_key(pmin_idx):
                E[pmin_idx].add(len(V))
            else:
                E[pmin_idx] = set([len(V)])   
            edge_C[pmin_idx,len(V)] = min_edge_c  
            cumulative_last = cmin     
            pnew_idx = len(V)
            V.append(pnew)
            C.append(c_new)
            parents[pnew_idx] = pmin_idx
            """
            Re-wire the tree
            """
            for en,p_idx in enumerate(Pnear_idx):
                # so if the near nodes, have children
                #parent
                if parents.has_key(p_idx):
                    p = V[p_idx]
                    c_near = C[p_idx]
                    e_c = self.cost_manager.edge_cost(c_near,c_new,p,pnew)
                    c = cumulative_last + e_c
                    if (self.segment_safe(p,pnew) is True and 
                        c < cumulative_costs[en]):
                        E[parents[p_idx]].remove(p_idx)
                        edge_C.pop(parents[p_idx],p_idx)
                        edge_C[pnew_idx,p_idx] = e_c
                        parents[p_idx] = pnew_idx
                        if E.has_key(pnew_idx):
                            E[pnew_idx].add(p_idx)
                        else:
                            E[pnew_idx] = set([p_idx])

            rrt_iter +=1
            if rrt_iter==len(cached_points):
                planning_done=True
                nbrs.fit(V)
                dist,points_near_goal = nbrs.radius_neighbors(goal_xy, self.goal_tolerance+0.1, return_distance = True)
                points_near_goal = points_near_goal[0]
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
        path = self.get_path(parents,V,m)
        path = self.smoothing(path)
        pt = path_to_pose(path)            
        print 'total time: ', time.time()-t1
        self.publish_rrt(V,E)        
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



    def astar(self):
        def get_score(point1,point2,goal):
            c1 = self.cost_manager.get_cost(point1,goal)
            c2 = self.cost_manager.get_cost(point2,goal)
            return self.cost_manager.edge_cost(c1,c2,point1,point2)
        def around(point,decimals=3):
            return np.around(point,decimals=decimals)
        def get_neighbours(point,res=0.2):
            nbrs = []
            for i in [-res,0,res]:
                for j in [-res,0,res]:
                    if i ==0 and j==0:
                        pass
                    else:
                        nbrs.append(np.around(point+np.array([i,j]),decimals=3))
            return nbrs

        def get_astar_path(parents,last,goal_xy):
            current = last
            path = [np.array(goal_xy),np.array(current)]
            at_root=False
            while at_root==False:
                next = parents[str(current)]
                current = next
                if next ==None:
                    at_root=True
                    break
                else:
                    path.append(np.array(next))
            return path

        """
        A* Algorithm
        """

        probot = np.around(np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y]),decimals=3)
        goal_xy = np.around(np.array([self._goal.pose.position.x,self._goal.pose.position.y]),decimals=3)

        resolution = self.astar_res # resolution in m
        height = self._navmap.info.height*self._navmap.info.resolution
        width  = self._navmap.info.width*self._navmap.info.resolution
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y

        closedSet = []
        openSet = Q.PriorityQueue()
        openSet.put((0,probot))

        cost_so_far = {str(probot):0}
        parents = {str(probot):None} # The parent of key is value
        loops = 0
        t1 = time.time()
        while not openSet.empty():
            current = openSet.get()[1]
            #marker = self.make_sample_marker(current)
            #self.samp_point_pub.publish(marker)
            if np.linalg.norm(current-goal_xy)<resolution:
                print "TIME TAKEN", time.time()-t1
                path = get_astar_path(parents,current,goal_xy)
                pt = path_to_pose(path)
                self._path_pub.publish(pt)
                print "COST A*",cost_so_far[str(current)]
                return pt,path
                break
            neighbours = get_neighbours(current,res = resolution) 
            for n,nbr in enumerate(neighbours):
                if self.segment_safe(current,nbr):
                    tentative_cost = cost_so_far[str(current)] + get_score(current,nbr,goal_xy)
                    if cost_so_far.has_key(str(nbr))==False or tentative_cost<cost_so_far[str(nbr)]:
                        cost_so_far[str(nbr)] = tentative_cost
                        priority = tentative_cost + get_score(nbr,goal_xy,goal_xy) + np.random.normal(size=1,scale=0.1)[0]
                        openSet.put((priority,nbr))

                        parents[str(nbr)] = current



    def publish_rrt(self, V,E):
        pt = Path()
        pt.header.frame_id = '/map'
        path = []
        vis = set()
        self.gen_path(0, 0, V, E, path, vis)
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

        
    def gen_path(self, ix, p_ix, V, E, path, vis ):
        path.append(V[ix])
        vis.add(ix)
        if E.has_key(ix):
            for c in E[ix]:
                if c not in vis:
                    self.gen_path(c, ix, V, E, path, vis )
        path.append(V[p_ix])
   
    def steer(self, org, dst):
        alpha = atan2(dst[1]-org[1],
                      dst[0]-org[0])
        new = org + self._rrt_eta*np.array([cos(alpha),sin(alpha)])
        return new
    def segment_safe(self, org, dst):
        org_idx = self.point_to_pixels(np.array(org))
        dst_idx = self.point_to_pixels(np.array(dst))
        v1 = self.distance_transform_search(org_idx, dst_idx)
        return v1
    def point_to_pixels(self,point):
        idx = np.array([int((point[0] - self.origin[0])/self.res),int((-point[1] + self.origin[1])/self.res)])
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
        r = np.random.randint(0,len(self._freecells),1)
        idx = self._freecells[r[0]]
        return pixel_to_point(idx,self._navmap)
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
    def sample_goal_bias(self,goal_xy,bias = 0.1):
        # sample a gaussian around the goal with probability bias
        point = [0,0]
        if np.random.binomial(1,bias)==0:
            point =  self.sample_free_uniform()
        else:
            noise = np.random.normal(size=2)
            point = goal_xy+noise
        return point
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
    rospy.init_node('entropy_motion_planner')
    m = MotionPlanner()
    rospy.spin()
