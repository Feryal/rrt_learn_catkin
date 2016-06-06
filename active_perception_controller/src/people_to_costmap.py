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
        self.people_pub = rospy.Subscriber("person_poses",
                                            PersonArray,
                                            self.ppl_cb,
                                            queue_size=1)
        self.pc_pub = rospy.Publisher("person_point_cloud",PointCloud,queue_size=3)
        self.p=PointCloud()
        self.tm = rospy.Timer(rospy.Duration(0.1), self.pub_ppl)
    def ppl_cb(self,msg):
        self.p = PointCloud()
        self.p.header.frame_id='map'
        self.p.header.stamp = rospy.get_rostime()
        chan = ChannelFloat32()
        chan.name = "intensity"
        for n,i in enumerate(msg.persons):
            self.p.points.append(i.pose.position)
            self.p.points[n].z=0.0
            chan.values.append(100)
        self.p.channels.append(chan)
    def pub_ppl(self,event):
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

