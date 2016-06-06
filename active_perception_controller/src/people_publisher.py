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
import rosbag
from data_structures import person,path_container



TELEOP = 1
AUTO = 2

class People_Publisher(object):
    def __init__(self,people_paths_directory,static = True):
        self.ppl_paths = fn.pickle_loader(people_paths_directory+"/ppl_paths.pkl")
        self.static = static
        self.person_publisher = rospy.Publisher("person_poses",PersonArray,queue_size = 10)
        self.marker_publisher = rospy.Publisher("person_markers",MarkerArray,queue_size = 10)
        self.no_of_people = len(self.ppl_paths)
        rospy.sleep(0.5)
        self.initialise_person_markers()
        if static ==False:
            self.dynamic_paths = []
            self.publish_frequency = 5 # messages per second
            self.sleeptime = 1./self.publish_frequency
            self.velocity=0.1
            for n,i in enumerate(self.ppl_paths):
                i.persons[0].vel=self.velocity
                i.persons[0].id=n
                p = self.get_path(i)
                self.dynamic_paths.append(p)
        #     # subscriber to the people_pose topic to send positions and velocities to stage
        #     # publisher to the people_pose topic to recoevep positions and velocities from stage
        
        #pdb.set_trace()
    def initialise_person_markers(self):
        self.person_markers = MarkerArray()
        p_array = PersonArray()
        for i in range(self.no_of_people):
            marker = Marker()
            p = Person();p.id=i;p.header.stamp = rospy.get_rostime()
            marker.type = Marker.ARROW
            marker.header.stamp=rospy.get_rostime()
            marker.header.frame_id="map"
            marker.id=i
            marker.pose = self.ppl_paths[i].persons[0].pose
            marker.pose.position.z = 0.7
            marker.scale.x = 0.6;marker.scale.y=0.2;marker.scale.z=0.2
            marker.color.a=1.0;marker.color.b=1.0
            p.pose = self.ppl_paths[i].persons[0].pose;
            self.person_markers.markers.append(marker)
            p_array.persons.append(p)
        self.person_publisher.publish(p_array)
        self.marker_publisher.publish(self.person_markers)

    def publish_markers(self,person_array):
        for i in range(len(person_array.persons)):
            self.person_markers.markers[i].header.stamp=rospy.get_rostime()
            self.person_markers.markers[i].header.frame_id="map"
            self.person_markers.markers[i].id=i
            self.person_markers.markers[i].pose = person_array.persons[i].pose
            self.person_markers.markers[i].pose.position.z = 0.7
        self.marker_publisher.publish(self.person_markers)
    def people_loop(self):
            counters = [0]*self.no_of_people
            reverse = [False]*self.no_of_people
            while True:
                array = PersonArray()
                for i in range(self.no_of_people):
                    if reverse[i]==True:
                        counters[i]-=1
                    elif reverse[i]==False:
                        counters[i]+=1
                    if counters[i] == len(self.dynamic_paths[i].persons):
                        reverse[i]=True
                        counters[i]-=1
                    elif counters[i]==0:
                        reverse[i]=False
                    if reverse[i]==True:
                        temp = copy.deepcopy(self.dynamic_paths[i].persons[counters[i]])
                        z =  temp.pose.orientation.z
                        w =  temp.pose.orientation.w 
                        temp.pose.orientation.z = -w
                        temp.pose.orientation.w = -z
                        array.persons.append(temp)
                    else:
                        array.persons.append(self.dynamic_paths[i].persons[counters[i]])
                self.person_publisher.publish(array)
                self.publish_markers(array)
                #self.publish_path_point(paths[0])
                rospy.sleep(self.sleeptime)
    def get_path(self,loaded_paths):
        # checkpoints are given as a list of shape [checkpoints,2]
        full_path = PersonArray()
        increments = self.velocity/self.publish_frequency
        poses = loaded_paths.persons
        for i in range(len(poses)-1):
            ex = np.array([poses[i].pose.position.x , poses[i].pose.position.y] )
            goua = np.array([poses[i+1].pose.position.x , poses[i+1].pose.position.y] )
            vector =goua-ex
            orientation =np.arctan2(vector[1],vector[0])
            length = np.linalg.norm(vector)
            num_of_points = length/increments
            x = np.linspace(0,vector[0],num_of_points) + poses[i].pose.position.x 
            y = np.linspace(0,vector[1],num_of_points) + poses[i].pose.position.y
            for j,k in zip(x,y):
                p = Person()
                p.pose.position.x = j
                p.pose.position.y = k
                quatern = tf.transformations.quaternion_from_euler(0,0,orientation)
                p.pose.orientation.z = quatern[-2];p.pose.orientation.w = quatern[-1]
                p.vel = poses[0].vel
                p.id = poses[0].id
                full_path.persons.append(p)
        return full_path
    def publish_random_position(self):
        p_array = PersonArray()
        for i in range(len(self.ppl_paths)):
            p = Person()
            p.header.frame_id = "map"
            p.header.stamp = rospy.get_rostime()
            rand = np.random.randint(low = 0,high =len(self.ppl_paths[i].persons) ,size = 1)
            p.pose = self.ppl_paths[i].persons[rand].pose
            p.vel = 0# random points are always for 0 velocity.
            p_array.persons.append(p)
        p_array.header.frame_id="map"
        p_array.header.stamp=rospy.get_rostime()
        for i in range(2):
            self.person_publisher.publish(p_array)
            self.publish_markers(p_array)
            rospy.sleep(0.1)

def listener():
    rospy.init_node('l',anonymous=True)

    p = People_Publisher("test1",static=False)
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
    arguement_list = sys.argv
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()
    #data = fn.pickle_loader("collected_data.pkl")
    #pdb.set_trace()

