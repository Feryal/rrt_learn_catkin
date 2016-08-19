#!/usr/bin/env python 
import pdb
import sys
import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import PointStamped
from active_perception_controller.msg import Person,PersonArray
import helper_functions as fn

class People_To_Costmap(object):
    def __init__(self):
        self.people_pub = rospy.Publisher("person_poses",
                                            PersonArray,
                                            queue_size=1)
        self.sub1 = rospy.Subscriber("Person1/poseStamped",PoseStamped,self.cb1)
        self.sub2 = rospy.Subscriber("Person2/poseStamped",PoseStamped,self.cb2)
        self.sub3 = rospy.Subscriber("Person3/poseStamped",PoseStamped,self.cb3)
        self.p=PersonArray()
        #self.tm = rospy.Timer(rospy.Duration(0.1), self.pub_ppl)
    def self.cb1(msg):

    def to_person(poseStamped):

    def people_pub(self,event):
        self.p.header.stamp =rospy.get_rostime()
        self.p.header.frame_id = 'map'
        self.pc_pub.publish(self.p)
def listener():
    rospy.init_node('ppl_to_costmap',anonymous=True)
    p = People_To_Costmap()
    rospy.spin()
if __name__ == '__main__':
    # System arguements for experiment builder = operation mode (teleop or auto). folder to save experiments. people: (static, moving)
    listener()

