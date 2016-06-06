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
import os
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Pose
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from active_perception_controller.msg import Person,PersonArray
from nav_msgs.msg import Odometry
import helper_functions as fn
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import LaserScan
from active_perception_controller.srv import positionChange
import rosbag
from data_structures import person,path_container

class Robot_Position_Manager_test(object):
    def __init__(self,goal_directory):
        self.goals = fn.pickle_loader(goal_directory+"/goals.pkl")
        self.set_initial_position()

    def set_initial_position(self):
        self.goal_index=0
        self.construct_goal(self.goals[0][0],self.goals[0][1])
        rospy.wait_for_service('change_robot_position')
        try:
            pc = rospy.ServiceProxy('change_robot_position', positionChange)
            pc(self.current_goal.target_pose.pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def construct_goal(self,x,y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        orient = np.random.uniform(-np.pi,np.pi)
        out = tf.transformations.quaternion_from_euler(0,0,orient)
        goal.target_pose.pose.orientation.z=out[-2]
        goal.target_pose.pose.orientation.w=out[-1]
        self.current_goal = goal
def listener():
    rospy.init_node('l',anonymous=True)
    path = os.path.dirname(os.path.abspath(__file__))
    name = path+"/"+"test1"
    p = Robot_Position_Manager_test(name)
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()
    #data = fn.pickle_loader("collected_data.pkl")
    #pdb.set_trace()

