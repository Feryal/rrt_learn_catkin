#!/usr/bin/env python 
import pdb
import sys
import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import PointStamped,PoseStamped
from active_perception_controller.msg import Person,PersonArray
import helper_functions as fn

class Mocap2Costlib(object):
    # What do we do?
    def __init__(self):


        self.listener = tf.TransformListener()
        got_transform = False
        while got_transform is False:
            try:
                self.trans,self.rot= self.listener.lookupTransform("world","/map",rospy.Time.now())
                got_transform = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.people_pub = rospy.Publisher("person_poses",
                                            PersonArray,
                                            queue_size=1)
        self.sub1 = rospy.Subscriber("/vrpn_client_node/Person1/pose",PoseStamped,self.cb1)
        self.sub2 = rospy.Subscriber("/vrpn_client_node/Person2/pose",PoseStamped,self.cb2)
        self.sub3 = rospy.Subscriber("/vrpn_client_node/Person3/pose",PoseStamped,self.cb3)
        self.p_array=PersonArray()


        self.people_local = [None,None,None]
        self.ppl_pub = rospy.Timer(rospy.Duration(0.1), self.publish_people_cb)
        self.timeout1 = rospy.Timer(rospy.Duration(0.4), self.cb_to1)
        self.timeout2 = rospy.Timer(rospy.Duration(0.4), self.cb_to2)
        self.timeout3 = rospy.Timer(rospy.Duration(0.4), self.cb_to3)
    def cb1(self,msg):
        person = self.to_person(msg,1)
        self.people_local[0] = person
        self.timeout1.shutdown()
        self.timeout1.run()
    def cb2(self,msg):
        person = self.to_person(msg,2)
        self.people_local[1] = person
        self.timeout2.shutdown()
        self.timeout2.run()
    def cb3(self,msg):
        person = self.to_person(msg,3)
        self.people_local[2] = person
        self.timeout3.shutdown()
        self.timeout3.run()
    def cb_to1(self,msg):
        self.people_local[0]=None
    def cb_to2(self,msg):
        self.people_local[1]=None
    def cb_to3(self,msg):
        self.people_local[2]=None

    def to_person(self,poseStamped,ide):
        p = Person()
        p.pose = poseStamped.pose
        p.pose.position.x += self.trans[0];p.pose.position.y += self.trans[1];p.pose.position.z += self.trans[2];
        p.pose.orientation.x += self.rot[0];p.pose.orientation.y += self.rot[1];p.pose.orientation.z += self.rot[2];p.pose.orientation.w += self.rot[3];
        p.id = ide
        p.header.frame_id = "map"
        p.header.stamp = rospy.get_rostime()
        return p
    def publish_people_cb(self,event):
        p_array = PersonArray()
        for n,i in enumerate(self.people_local):
            if i != None:
                p_array.persons.append(i)
        p_array.header.stamp =rospy.get_rostime()
        p_array.header.frame_id = 'map'
        self.people_pub.publish(p_array)
def listener():
    rospy.init_node('mocap_2_costlib',anonymous=True)
    mocap2costlib = Mocap2Costlib()
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()

