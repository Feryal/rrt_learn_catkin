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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Pose
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from active_perception_controller.msg import Person,PersonArray
from nav_msgs.msg import Odometry
import helper_functions as fn
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import LaserScan
from people_publisher import People_Publisher
from std_msgs.msg import Float32MultiArray
import rosbag
from active_perception_controller.srv import positionChange,ppl_positionChange
from data_structures import person,path_container
import os


TELEOP = 1
AUTO = 2

class Config_Publisher(object):
    def __init__(self,mode,goal_directory,people_static = True):
        # load the goals from the correct file.    
        self.goals = fn.pickle_loader(goal_directory+"/goals.pkl")
        self.current_goal = None
        self.max_goals =  rospy.get_param("~datapoints_to_collect", 5)
        self.goals_sent = 0
        self.succeded = 0
        self.goal_index = 100
        self.mode=1
        self.goal_directory = goal_directory
        self.weights = None
        fn.pickle_saver(self.weights,self.goal_directory+"/weights.pkl")
        fn.make_dir(goal_directory+'/traj_data/')
        rospy.on_shutdown(self.shutdown)
        #first bag file to record

        self.bag = rosbag.Bag(goal_directory+'/traj_data/test.bag','w')
        self.tf_listener = tf.TransformListener()
        self.people_static = people_static
        self.set_initial_position() # initial position set based on the first place in goals
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
               'SUCCEEDED', 'ABORTED', 'REJECTED',
               'PREEMPTING', 'RECALLING', 'RECALLED',
               'LOST']
        # action client so send goals
        self.move_base =actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.goal_pub = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
        # Dealing with cases when people are not detected.
        self.new_people_message = False;self.people_message_timeout = 0;
        self.people_latest = PersonArray()
        self.new_bag = False
        if self.mode == AUTO:
        #connect to movebase
            connected = False
            while not connected:
                try:
                    self.move_base.wait_for_server(rospy.Duration(15))
                    connected = True
                except rospy.ServiceException:
                    rospy.logwarn("could not connect. etc") 

        #initialise extra markers and publisers for experiment

        self.initialise_marker()   

        self.people_publisher = People_Publisher(self.goal_directory,static = True)
        self.pub1 = rospy.Publisher('visualise_goal',Marker,queue_size = 200)
        self.pub2 = rospy.Publisher('visualise_orient',Marker,queue_size = 200)
        # Robot position and people subscribers
        self.position_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.position_callback)
        self.pos_marker_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.pos_marker_callback)
        self.people_sub = rospy.Subscriber("/person_poses",PersonArray,self.people_callback)
        self.weights_sub = rospy.Subscriber("weights",Float32MultiArray,self.weights_cb)
        rospy.sleep(1.)
        print "ABOUT TO ENTER THE LOOOOOOOPP__________________________________________________"
        self.random_goal_loop()

    def set_initial_position(self):
        self.goal_index=0
        init = self.construct_goal(self.goals[0][0],self.goals[0][1])
        rospy.wait_for_service('change_robot_position')
        try:
            pc = rospy.ServiceProxy('change_robot_position', positionChange)
            pc(init.pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    def get_random_goal(self):
        new_goal = self.goal_index
        if len(self.goals)==1:
            print "Only one goal"
            new_goal=0
        else:
            while new_goal == self.goal_index:
                print "im stuck here"
                new_goal = np.random.randint(low = 0,high = len(self.goals))
        self.goal_index = new_goal
    def random_goal_loop(self):
        while self.goals_sent!=self.max_goals:
            if self.goals_sent>0:
                self.new_bag = True
                self.bag.close()
                self.bag = rosbag.Bag(self.goal_directory+'/traj_data/test'+str(self.goals_sent)+".bag",'w')
                self.new_bag = False
            #this updates the current goal index
            self.get_random_goal()
            self.current_goal = self.construct_goal(self.goals[self.goal_index][0],self.goals[self.goal_index][1])
            #self.goal_pub.publish(self.current_goal)
            self.bag.write("goal",self.current_goal)
            self.visualise_goal()
            # Publish random people position
            if self.people_static == True:
                self.people_publisher.publish_random_position()
            rospy.sleep(0.5)
            # if mode is automatic connect to move base
            if self.mode ==AUTO:
                connected = False
                while not connected:
                    try:
                        self.move_base.wait_for_server(rospy.Duration(50))
                        connected = True
                    except rospy.ServiceException:
                        rospy.logwarn("could not connect. etc")
                rospy.sleep(2.)
                self.move_base.send_goal(self.current_goal)
                print "GOAL SENT",self.current_goal
                self.goals_sent+=1
                #state = self.move_base.get_state()
                if self.people_static == False:
                    while self.goal_states[state] != 'SUCCEEDED':
                        for i in self.people_publisher.paths:
                            self.people_publisher.publish_path_point(i)
                        rospy.sleep(self.people_publisher.sleeptime)
                else:
                    goal_reached=False
                    print "waiting for success"
                    while not goal_reached:
                        goal_reached = self.goal_check(0.4)
                    print "goal succeded"
                    self.succeded+=1   
                    rospy.sleep(0.2)  
                #     finished_within_time = self.move_base.wait_for_result()                 
                # if self.goal_states[state] == 'SUCCEEDED':
                #     rospy.loginfo("Goal succeeded!")
                #     self.succeded += 1
                #     rospy.loginfo("State:" + str(state))
            # If in teleop mode simply send the target and await execution with a certain tolerance on accuracy.
            elif self.mode == TELEOP:
                self.goals_sent+=1
                goal_reached = False
                while not goal_reached:
                    if self.people_static == False:
                        for i in self.people_publisher.paths:
                            self.people_publisher.publish_path_point(i)
                        rospy.sleep(self.people_publisher.sleeptime)
                    else:
                        goal_reached = self.goal_check(0.4)
                        rospy.sleep(0.1)
                self.succeded += 1
                print "Goal reached"
            #else:
              #rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

    def goal_check(self,tolerance):
        diff_x = np.absolute(self.current_goal.pose.position.x - self.pose_latest.pose.position.x)
        diff_y = np.absolute(self.current_goal.pose.position.y - self.pose_latest.pose.position.y)
        #diff_z = np.absolute(self.current_goal.target_pose.pose.orientation.z - self.pose_latest.pose.orientation.z)
        #diff_w = np.absolute(self.current_goal.target_pose.pose.orientation.w - self.pose_latest.pose.orientation.w)
        if diff_x<tolerance and diff_y<tolerance:
            return True
        else:
            return False
    def construct_goal(self,x,y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        orient = np.random.uniform(-np.pi,np.pi)
        out = tf.transformations.quaternion_from_euler(0,0,orient)
        goal.pose.orientation.z=out[-2]
        goal.pose.orientation.w=out[-1]
        return goal

    def initialise_marker(self):
        self.goal_marker = Marker()
        self.goal_marker.type = Marker.CUBE
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.pose.position.z = 2.5
        self.goal_marker.scale.x = 0.3;self.goal_marker.scale.y=0.3;self.goal_marker.scale.z=0.3
        self.goal_marker.color.a=0.0;self.goal_marker.color.g=1.0
        self.orient_marker = Marker()
        self.orient_marker.type = Marker.ARROW
        self.orient_marker.header.frame_id = "base_link"
        self.orient_marker.pose.position.z = 0.7
        self.orient_marker.scale.x = 0.6;self.orient_marker.scale.y=0.2;self.orient_marker.scale.z=0.2
        self.orient_marker.color.a=1.0;self.orient_marker.color.b=1.0
    def visualise_goal(self):
        if self.goal_index != None:
            self.goal_marker.color.a=1.0
            self.goal_marker.pose = self.current_goal.pose
        self.pub1.publish(self.goal_marker)
    def pos_marker_callback(self,msg):
        self.pub2.publish(self.orient_marker)
    def position_callback(self,msg):
        #quatern = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        #self.pose_latest = [msg.pose.pose.position.x,msg.pose.pose.position.y,tf.transformations.euler_from_quaternion(quatern)]
        #self.position_msg_latest = msg
        self.pose_latest = PoseStamped()
        self.pose_latest.header = msg.header;
        self.pose_latest.pose = msg.pose.pose
        if self.new_bag == False:
            self.bag.write("robot",self.pose_latest)
            self.bag.write("people",self.people_latest)
        #print "CURRENT position", self.possition_latest
    def people_callback(self,msg):
        self.people_latest = msg
    def weights_cb(self,msg):
        self.weights = msg.data
        fn.pickle_saver(self.weights,self.goal_directory+"/weights.pkl")

    def shutdown(self):
        rospy.loginfo("Stopping Node...")
        if self.mode == AUTO:
            self.move_base.cancel_goal()
            rospy.sleep(1)
        rospy.signal_shutdown("shutdown")

def listener():
    print "GOHRERE"
    rospy.init_node('l',anonymous=True)
    name = rospy.get_param("~experiment_name", "test")
    path = os.path.dirname(os.path.abspath(__file__))
    name = path+"/"+name

    mode = rospy.get_param("~drive_mode", 2)
    people_static = rospy.get_param("~people_static", True)

    if mode==2:
        mode=AUTO
    else:
        mode=TELEOP
    print "MODE IS", mode
    p = Config_Publisher(mode,name,people_static=people_static)

    #goal_collector = Config_Publisher(AUTO,"LesArcades_no_chairs")
    #people_collector = People_Collector("people_collector_test/test1")
    # if people=="static":
    #     people_stat = True
    # elif people=="moving":
    #     people_stat =False
    # else:
    #     print "Unknown people command. exiting"

    # if mode=="auto":
    #     operation_mode = AUTO 
    # elif mode=="teleop":
    #     operation_mode = TELEOP
    # else:
    #     print "Unknown people command. exiting"
    # ppe = Config_Publisher(operation_mode,experiment_name,people_static =people_stat)
    # while not rospy.is_shutdown():
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()
    #data = fn.pickle_loader("collected_data.pkl")
    #pdb.set_trace()

