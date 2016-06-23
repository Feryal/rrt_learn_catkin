#!/usr/bin/env python
import rospy
import math
import actionlib
import thread
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy


CLOSE = 0.85
STANDARD = 1.4
FAR = 2.0
max_lin_speed = 0.5
max_rot_speed = 0.5
max_height = 1.410
min_height = 1.160

class joycontroller:
        def __init__(self):
                self.enabled = False
                self.approach_active = False
                self.calib_info = False
                self.calib_x = None
		self.button_is_pressed = False
                self.prev = Twist()
		self.vel_command = Twist()
                self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    		self.sub = rospy.Subscriber("joy", Joy, self.callback)
                #self.soundhandle = SoundClient()
                #Approach actionlib client
                #self.approach_client = actionlib.SimpleActionClient('approach_person', ApproachPersonAction)
                
 #               while not self.approach_client.wait_for_server(rospy.Duration(2)):
 #                       rospy.logwarn("Trying to connect to 'approach_person' action")
 #                       rospy.sleep(1)
 #               rospy.loginfo("Joystick bridge successfully connected to 'approach_person' action server.")

    		self.publisher()

	def callback(self, data):
                if self.calib_x is None:
                        if not self.calib_info:
                                rospy.logwarn("Not calibrated. Center the joystick and press the trigger")
                                self.calib_info = True
                                return
                        if data.buttons[0] and data.axes[1] != 0:
                                rospy.logwarn("Calibration saved. Joystick is active. Press the trigger again to deactivate.")
                                self.calib_x = data.axes[1]
                                self.enabled = True
                                return
                else:
                        twist = Twist()
                        dx = data.axes[1]
                        a = 0
                        b = 0
                        if dx >= self.calib_x:
                                a = -1.0/self.calib_x
                                b = 1.0
                        else:
                                a = 1.0/(self.calib_x + 1.0)
                                b = a - 1.0

                        twist.linear.x = max_lin_speed*(a*dx + b)

                        if abs(twist.linear.x) < 0.1:
                                twist.linear.x = 0.0
                        
                        twist.angular.z = max_rot_speed*data.axes[0]
                        self.vel_command = twist

                        height = (0.5*data.axes[3] + 0.5)*(max_height-min_height) + min_height
			#Sound buttons
#                        if not self.button_is_pressed and data.buttons[2]:
  #                              rospy.loginfo("Playing sound.")
 #                       	self.soundhandle.play(4)
#				self.button_is_pressed = True
 #                       elif self.button_is_pressed and not data.buttons[2]:
#				self.button_is_pressed = False				

                        if data.buttons[0]:
                                if self.enabled:
                                        rospy.loginfo("Joystick disabled.")
                                        self.enabled = False
                                else:
                                        rospy.loginfo("Joystick enabled.")
                                        self.enabled = True

#                        if not self.approach_active:
#                                if data.buttons[7]:
#                                        rospy.logwarn("=== Triggered Autonomous Approach - Close (Press again to cancel) ===")
#                                        self.approach_active = True
#                                        self.enabled = False
#                                        thread.start_new_thread(self.send_approach, (CLOSE,))
#                                if data.buttons[8]:
#                                        rospy.logwarn("=== Triggered Autonomous Approach - Standard (Press again to cancel) ===")
#                                        self.approach_active = True
#                                        self.enabled = False
#                                        thread.start_new_thread(self.send_approach, (STANDARD,))
#                                if data.buttons[9]:
#                                        rospy.logwarn("=== Triggered Autonomous Approach - Far (Press again to cancel) ===")
#                                        thread.start_new_thread(self.send_approach, (FAR,))
#                                        self.approach_active = True
#                                        self.enabled = False
#                        else:
#                                if data.buttons[7] or data.buttons[8] or data.buttons[9]:
#                                        rospy.logwarn("=== Cancelled Autonomous Approach ===")
#                                        self.approach_client.cancel_all_goals()
#                                        self.approach_active = False


#        def send_approach(self, ref):
#                goal = ApproachPersonGoal(target_distance = ref)
#                self.approach_client.send_goal(goal)
#                self.approach_client.wait_for_result()
#                rospy.loginfo("=== Approach Action Terminated ===")
#                self.approach_active = False

	def publisher(self):
    		rate = rospy.Rate(10)
    		while not rospy.is_shutdown():
                        if self.enabled:
                                self.cmd_pub.publish(self.vel_command)
                        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joy_to_giraff')
    joy = joycontroller()
