#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
# from move_and_grasp.msg import ObjectPosition
from kamerider_image_msgs import ObjectPosition

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        self.linear_speed = 0.15
        
        ############### Set the travel distance in meters
        self.goal_distance = 0

        # Set the rotation speed in radians per second
        self.angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(1.0)
        
        # Set the rotation angle to Pi radians (180 degrees)
        # goal_angle = pi
        # Set the required position for arm to grasp
        # You can find the required_pos in the moveit_ik_demos.py
        self.required_pos = Point()
        # for ik demo
        # self.required_pos.x = 0.3756
        # self.required_pos.y  = 0.01619
        # for fk  demo
        self.required_pos.x = 0.348105493
        self.required_pos.y  = 0.00053959

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        rospy.Subscriber("/kamerider_image/object_position_base", ObjectPosition, self.Pointcloud_Callback)
        #Set a publisher to mark the end
        self.pub_to_arm_topic_name = 'adjust_to_arm'
        self.pub_to_arm = rospy.Publisher(self.pub_to_arm_topic_name, String, queue_size=1)


    def Pointcloud_Callback(self, target_pos):       
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        #Compute the move distance
        dis_x = target_pos.x - self.required_pos.x 
        dis_y = target_pos.y - self.required_pos.y
        if dis_x > 0.75:
	    print("dis_x > 0.75, pls shut down and try again")
            dis_x = 0
        if dis_y >0.75:
            print("dis_y > 0.75, pls shut down and try again")
            dis_y = 0
        
        # Initialize the position variable as a Point type
        position = Point()
        # Get the starting position values     
        (position, rotation) = self.get_odom()
        #wait for 10s
        rospy.sleep(10)
        #move on the y axis
        target_radius = pi/2
        self.move_around(target_radius,rotation)
        (position, rotation) = self.get_odom()
        self.go_straight(dis_y,position)
        (position, rotation) = self.get_odom()
        self.move_around(-target_radius,rotation)
        
        #move on the x axis
        (position, rotation) = self.get_odom()
        self.go_straight(dis_x,position)             
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        self.pub_to_arm.publish("start_grasp")
    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    def go_straight(self,goal_distance,start_pos):
        move_cmd = Twist()
                
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        
        if goal_distance < 0:
            move_cmd.linear.x = -self.linear_speed
            goal_distance = -goal_distance
                    
        x_start = start_pos.x
        y_start = start_pos.y
        
        # Keep track of the distance traveled
        traveled_distance = 0
        
        # Enter the loop to move along a side
        while traveled_distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            self.r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the Euclidean distance from the start
            traveled_distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

        # Stop the robot 
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("go {} meters long end!".format(goal_distance))
        return


    def move_around(self,goal_radius,start_rotation):
        move_cmd = Twist()

        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        if goal_radius < 0:
            move_cmd.angular.z = -self.angular_speed
            goal_radius = -goal_radius

        # Track the last angle measured
        last_angle = start_rotation
        
        # Track how far we have turned
        turn_angle = 0
        
        while abs(turn_angle + self.angular_tolerance) < abs(goal_radius) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
            
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
            
        # Stop the robot 
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("Turn {} radius  end!".format(goal_radius))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':    
    OutAndBack()
    rospy.spin()

