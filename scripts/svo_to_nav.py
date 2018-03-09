#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class NavPublisher():
    def __init__(self);
        # TODO: initialize class vars for velocity and covariance
        

        rospy.init_node('bebop_svo_to_nav', anonymous=False)

        # subscribers (TODO)
        rospy.Subscriber('svo/pose_cam/0', PoseStamped, self.svo_callback)
        rospy.Subscriber('bebop/odom', Odometry, self.bebop_callback)

        # publishers (TODO: publish to /vo for robot pose ekf)

        rospy.pub_vo = rospy.Publisher('/vo', Odometry, queue_size=10)

    def svo_callback(self, data):
        # converts to nav_msg odometry type msg and publish

    def bebop_callback(self, data):
        # gets data for velocity and covariance and store it for svo

    def shutdown(self):
        rospy.loginfo("Shutting down bebop_svo_to_nav node")


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting bebop_svo_to_nav node.")
        NavPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Terminating bebop_svo_to_nav node.")

