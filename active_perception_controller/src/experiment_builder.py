#!/usr/bin/env python 
from __future__ import with_statement 
import rospy
import math
import numpy as np
import helper_functions as fn
import pdb
import os
from active_perception_controller.msg import Person,PersonArray
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Pose,PointStamped
import tf
from visualization_msgs.msg import Marker,MarkerArray

class Goal_Collector(object):
    # this class listens to published points or RViZ and logs them down,
    # to be used as possible random goals for the RRT-IRL algorithm data collection.
    def __init__(self,name):
        # specify number of goals to collect
        self.number_of_goals = rospy.get_param("~number_of_goals", 3)
        print self.number_of_goals
        # save goals here
        self.goals = []
        # specify name of file to be save
        self.save_name = name
        # to stop spinning
        self.done = False
    def point_recieved(self,data):
        if self.done == False:
            self.goals.append([data.point.x,data.point.y])
            print "data_Save"
            print self.number_of_goals,len(self.goals)
            if len(self.goals) == self.number_of_goals:
                #save to directory
                fn.pickle_saver(self.goals,self.save_name+"/goals.pkl")
                print "DATA SAVED. WILL ADDING MORE GOALS WILL NOT HAVE AN EFFECT"
                self.done = True

class People_Collector(object):
    # this class listens to published points or RViZ and logs them down,
    # to be used as possible random goals for the RRT-IRL algorithm data collection.
    # TODO: put these settings for the checkpoints and number of people as params
    def __init__(self,name):
        # specify number of goals to collect
        self.checkpoints_per_person = rospy.get_param("~checkpoints_per_person", 3)
        # specify number of people
        self.no_of_people = rospy.get_param("~number_of_people", 3)
        self.save_name = name
        # create data containers
        self.point_pub = rospy.Publisher('all_clicked',MarkerArray,queue_size = 200)
        self.all_points = MarkerArray()
        self.people = []
        for i in range(self.no_of_people):
            p = PersonArray()
            self.people.append(p)
        # specify name of file to be save

        self.current_person = 0
        # to stop spinning
        self.done = False
    def point_recieved(self,data):
        if len(self.people[self.current_person].persons)!= self.checkpoints_per_person:
            p = Person()
            p.pose.position.x = data.point.x ; p.pose.position.y = data.point.y
            quatern = tf.transformations.quaternion_from_euler(0,0,np.random.uniform(-np.pi,np.pi,1))
            p.pose.orientation.z = quatern[-2];p.pose.orientation.w = quatern[-1]
            p.id = self.current_person ; p.vel=0 ;p.header.stamp=rospy.get_rostime();p.header.frame_id="map"
            self.people[self.current_person].persons.append(p)

            print "Registered point",data.point.x,data.point.y
            print "For Person",self.current_person
            statement = len(self.people[self.current_person].persons)== self.checkpoints_per_person
            if statement == True and self.current_person == self.no_of_people-1:
                fn.pickle_saver(self.people,self.save_name+"/ppl_paths.pkl")
                print "DATA SAVED.Adding more routes for people will not have an effect"
                rospy.signal_shutdown("Done collecting people. On to collecting goals")
            if len(self.people[self.current_person].persons)== self.checkpoints_per_person:
                print "DONE WITH THIS PERSON. New points will be registered for others or none"
        else:
            self.current_person+=1
            p = Person()
            p.pose.position.x = data.point.x ; p.pose.position.y = data.point.y
            quatern = tf.transformations.quaternion_from_euler(0,0,np.random.uniform(-np.pi,np.pi,1))
            p.pose.orientation.z = quatern[-2];p.pose.orientation.w = quatern[-1]
            p.id = self.current_person ; p.vel=0 ;p.header.stamp=rospy.get_rostime();p.header.frame_id="map"
            self.people[self.current_person].persons.append(p)
            print "NEW PERSON"
            print "Registered point",data.point.x,data.point.y
            print "For Person",self.current_person

        marker = Marker()
        marker.type = Marker.CUBE
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.pose.position.x = data.point.x;marker.pose.position.y = data.point.y;marker.pose.position.z = 2.5
        marker.scale.x = 0.3;marker.scale.y=0.3;marker.scale.z=0.3
        marker.color.a=1.0;marker.color.g=1.0
        marker.id = len(self.all_points.markers)
        self.all_points.markers.append(marker)
        self.point_pub.publish(self.all_points)
    def shutdown(self):
        print "Shutting down people collector"

class experiment_buider(object):
    def __init__(self,save_name):
        self.save_name = save_name
        self.people_collector = People_Collector(self.save_name)
        self.goal_collector = Goal_Collector(self.save_name)
        print "---------------GOALS SELECTION PHASE--------------------"
        self.goal_subscriber = rospy.Subscriber("/clicked_point",PointStamped,self.goal_callback)

    def goal_callback(self,msg):
        if self.goal_collector.done==False:
            self.goal_collector.point_recieved(msg)
            if self.goal_collector.done==True:
                print "---------------PEOPLE SELECTION PHASE--------------------"
        elif self.people_collector.done==False:
            self.people_collector.point_recieved(msg)
        else:
            rospy.signal_shutdown("Goal and people colection finished")
def listener():
    rospy.init_node('l',anonymous=True)
    name = rospy.get_param("~experiment_name", "test")
    path = os.path.dirname(os.path.abspath(__file__))
    name = path+"/"+name
    fn.make_dir(name)
    people_collector = experiment_buider(name)
    rospy.spin()
if __name__ == '__main__':
    listener()

