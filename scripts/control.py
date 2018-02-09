#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from std_msgs.msg import Empty
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler

class BebopControl():
    def __init__(self):
        self.speed = 0.25
        self.land_initiated = False

        rospy.init_node('bebop_control', anonymous=False)

        #subscribers
        rospy.Subscriber('bebop/initiate_landing', Empty, self.land_callback)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        #publishers
        
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1, latch=True)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)

        # uncomment to enable takeoff on start here
        #takeoff = Empty()
        #self.pub_takeoff.publish(takeoff)

    def shutdown(self):
        rospy.loginfo("Shutting down bebop_control node.")
        land = Empty()
        self.pub_land.publish(land)
        rospy.sleep()

    # call backs for landing intiation
    def land_callback(self, data):
        if self.land_initiated == False:
            self.land_initiated = True
            rospy.loginfo("Initiating Landing Sequence.")

    # call back for marker pose
    def marker_callback(self, data):
        if self.land_initiated:
            print data.markers
    

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting BebopControl node.")
        BebopControl()
        rospy.spin()
    except rospy.ROSInterrupException:
        rospy.logerr("BebopControl node terminated.")