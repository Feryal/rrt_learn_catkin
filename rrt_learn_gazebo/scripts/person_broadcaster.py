#!/usr/bin/env python
import rospy
import tf
import thread
from geometry_msgs.msg import PointStamped


class PersonBroadcaster(object):
    def __init__(self):
        self._sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self._bcaster = tf.TransformBroadcaster()
        self._rate = rospy.Rate(20)
        self._point_1 = None
        self._point_2 = None
        thread.start_new_thread(self.broadcast_loop, ())

    def callback(self, msg):
        self._point_1 = (msg.point.x, msg.point.y, 0)
        self._point_2 = (msg.point.x + 1, msg.point.y, 0)

    def broadcast_loop(self):
        while not rospy.is_shutdown():
            if self._point_1 is not None:
                self._bcaster.sendTransform(self._point_1,
                                            (0, 0, 0, 1),
                                            rospy.Time.now(),
                                            "/Person_1_torso",
                                            "/odom")
                self._bcaster.sendTransform(self._point_2,
                                            (0, 0, 0, 1),
                                            rospy.Time.now(),
                                            "/Person_2_torso",
                                            "/odom")
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('person_broadcaster')
    pb = PersonBroadcaster()
    rospy.spin()
