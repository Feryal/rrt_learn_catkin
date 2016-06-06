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
from active_perception_controller.srv import positionChange,ppl_positionChange
import rosbag
from data_structures import person,path_container

class Robot_Position_Manager(object):
    def __init__(self):
        self.srv = rospy.Service('change_robot_position', positionChange, self.handle_position_change)
        self.ppl_srv = rospy.Service('change_people_position', ppl_positionChange, self.handle_ppl_position_change)
        self.gazebo_pub = rospy.Publisher("gazebo/set_model_state",ModelState,queue_size = 1)
        self.amcl_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size = 1)
        self.ppl_pub = rospy.Publisher("/person_poses",PersonArray,queue_size = 1)
        self.marker_pub = rospy.Publisher("/person_markers",MarkerArray,queue_size = 1)
        print "ALL STARTED"
    def handle_position_change(self,pose):
        model_state = ModelState()
        model_state.model_name = "teresa_robot"
        model_state.pose = pose.pose
        model_state.reference_frame="map"
        for i in range(25):
            self.gazebo_pub.publish(model_state)
            rospy.sleep(0.03)
        for i in range(25):
            amcl_pose = PoseWithCovarianceStamped()
            amcl_pose.pose.pose = pose.pose
            amcl_pose.header.frame_id = "map"
            amcl_pose.header.stamp = rospy.get_rostime()

            self.amcl_pub.publish(amcl_pose)
            rospy.sleep(0.03)
    def handle_ppl_position_change(self,ppl_array):
        person_markers = MarkerArray()
        p = PersonArray()
        p = ppl_array.ppl_array
        p.header.stamp = rospy.get_rostime()
        for n,i in enumerate(p.persons):
            i.header.stamp = rospy.get_rostime()
            i.id = int(n)
            marker = Marker()
            marker.type = Marker.ARROW
            marker.header.stamp=rospy.get_rostime()
            marker.header.frame_id="map"
            marker.id=n
            marker.pose = i.pose
            marker.pose.position.z = 0.7
            marker.scale.x = 0.6;marker.scale.y=0.2;marker.scale.z=0.2
            marker.color.a=1.0;marker.color.b=1.0
            person_markers.markers.append(marker)
        self.ppl_pub.publish(p)
        self.marker_pub.publish(person_markers)
        rospy.sleep(0.1)

def listener():
    rospy.init_node('l',anonymous=True)
    p = Robot_Position_Manager()
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()
    #data = fn.pickle_loader("collected_data.pkl")
    #pdb.set_trace()

