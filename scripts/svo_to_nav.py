#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry

class NavPublisher():
    def __init__(self):
        # initialize twist with covariance
        self.twist = TwistWithCovariance()

        rospy.init_node('bebop_svo_to_nav', anonymous=False)

        # subscribers (TODO)
        rospy.Subscriber('bebop/odom', Odometry, self.bebop_callback)
        rospy.Subscriber('svo/pose_imu', PoseWithCovarianceStamped, self.svo_callback)    

        # publishers (TODO: publish to /vo for robot pose ekf)

        self.pub_vo = rospy.Publisher('/vo', Odometry, queue_size=10)

    def svo_callback(self, data):
        # converts to nav_msg odometry type msg and publish
        odom = Odometry()
        odom.pose = data.pose
        odom.twist = self.twist

        self.pub_vo.publish(odom)

    def bebop_callback(self, data):
        # gets data for velocity and covariance and store it for svo
        self.twist = data.twist

    def shutdown(self):
        rospy.loginfo("Shutting down bebop_svo_to_nav node")


if __name__ == '__main__':
    try:
        NavPublisher()
        rospy.loginfo("Starting bebop_svo_to_nav node.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Terminating bebop_svo_to_nav node.")

