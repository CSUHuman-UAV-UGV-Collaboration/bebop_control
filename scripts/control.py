#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from std_msgs.msg import Empty
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class BebopControl():
    def __init__(self):
        self.speed = 0.02
        self.vert_speed = 0.25
        self.land_initiated = False
        self.angle_zone = 0.10
        self.position_zone = 0.05
        self.can_rotate = False
        self.angle_locked = False
        self.x_locked = False
        self.y_locked = False
        self.land_height = 0.5

        rospy.init_node('bebop_control', anonymous=False)

        #subscribers
        # !!! Change the callback function in marker subscriber to change landing behavior
        # !!! > self.marker_callback_simul or > self.marker_callback

        rospy.Subscriber('bebop/initiate_landing', Empty, self.land_callback)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback_simul)
        
        #publishers
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1, latch=True)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_camera_control = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

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
            cam_msg = Twist()
            cam_msg.angular.y = -90
            self.pub_camera_control.publish(cam_msg)
            rospy.sleep(4)
            rospy.loginfo("> Camera facing down.")

    def non_zero_vel(self, vel_msg):
        if (vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.linear.z == 0 and vel_msg.angular.z == 0):
            return False

        return True

    # call back for marker pose
    def marker_callback(self, data):
        
        if self.land_initiated:
            for marker in data.markers:
                vel_msg = Twist()
                q = marker.pose.pose.orientation
                roll,pitch,yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                p = marker.pose.pose.position
                print "position", p

                # rotate
                if self.can_rotate:
                    print "euler: ",roll,pitch,yaw
                    if yaw < (1.57 - self.angle_zone):
                        print "rotate counter clockwise"
                        vel_msg.angular.z = 0.1
                        self.angle_locked = False
                    elif yaw > (1.57 + self.angle_zone):
                        print "rotate clockwise"
                        vel_msg.angular.z = -0.1
                        self.angle_locked = False
                    else:
                        print "angle locked"
                        vel_msg.angular.z = 0
                        self.angle_locked = True

                # position

                # left-right
                if p.x > (0 + self.position_zone):
                    vel_msg.linear.y = -self.speed
                    self.x_locked = False
                elif p.x < (0 - self.position_zone):
                    vel_msg.linear.y = self.speed
                    self.x_locked = False
                else:
                    vel_msg.linear.y = 0
                    self.x_locked = True
                    print "x locked"

                # forward-backward
                if p.y > (0 + self.position_zone):
                    vel_msg.linear.x = -self.speed
                    self.y_locked = False
                elif p.y < (0 - self.position_zone):
                    vel_msg.linear.x = self.speed
                    self.y_locked = False
                else:
                    vel_msg.linear.x = 0
                    self.y_locked = True
                    print "y locked"

                if self.x_locked and self.y_locked:
                    self.can_rotate = True
                else:
                    self.can_rotate = False

                # descend
                if self.angle_locked == True and self.x_locked == True and self.y_locked == True:
                    if p.z > self.land_height:
                        vel_msg.linear.z = -(self.vert_speed)
                    else:
                        land = Empty()
                        self.pub_land.publish(land)
                        #self.initiate_landing = False

                self.pub_cmd_vel.publish(vel_msg)

    # new call back for marker
    def marker_callback_simul(self, data):
        
        if self.land_initiated:
            for marker in data.markers:
                #testing only
                self.can_rotate = True

                vel_msg = Twist()
                q = marker.pose.pose.orientation
                roll,pitch,yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                p = marker.pose.pose.position
                print "position", p

                # rotate
                if self.can_rotate:
                    print "euler: ",roll,pitch,yaw
                    if yaw < (1.57 - self.angle_zone):
                        print "rotate counter clockwise"
                        vel_msg.angular.z = 0.06
                        self.angle_locked = False
                    elif yaw > (1.57 + self.angle_zone):
                        print "rotate clockwise"
                        vel_msg.angular.z = -0.06
                        self.angle_locked = False
                    else:
                        print "angle locked"
                        vel_msg.angular.z = 0
                        self.angle_locked = True

                # position

                # left-right
                if p.x > (0 + self.position_zone):
                    vel_msg.linear.y = -self.speed
                    self.x_locked = False
                elif p.x < (0 - self.position_zone):
                    vel_msg.linear.y = self.speed
                    self.x_locked = False
                else:
                    vel_msg.linear.y = 0
                    self.x_locked = True
                    print "x locked"

                # forward-backward
                if p.y > (0 + self.position_zone):
                    vel_msg.linear.x = -self.speed
                    self.y_locked = False
                elif p.y < (0 - self.position_zone):
                    vel_msg.linear.x = self.speed
                    self.y_locked = False
                else:
                    vel_msg.linear.x = 0
                    self.y_locked = True
                    print "y locked"

                #if self.x_locked and self.y_locked:
                    #self.can_rotate = True
                #else:
                #    self.can_rotate = False

                # descend
                if self.angle_locked == True and self.x_locked == True and self.y_locked == True:
                    if p.z > self.land_height:
                        vel_msg.linear.z = -(self.vert_speed)
                    else:
                        land = Empty()
                        self.pub_land.publish(land)
                        #self.initiate_landing = False

                if(self.non_zero_vel(vel_msg)):
                    self.pub_cmd_vel.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting BebopControl node.")
        BebopControl()
        rospy.spin()
    except rospy.ROSInterrupException:
        rospy.logerr("BebopControl node terminated.")