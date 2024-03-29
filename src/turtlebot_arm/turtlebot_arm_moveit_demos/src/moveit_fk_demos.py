#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from std_msgs.msg import String

class MoveItDemo:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        rospy.Subscriber("/adjust_to_arm", String, self.armCallback)
        self.arm_pub = rospy.Publisher("/arm_to_control", String, queue_size=1)
        
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        self.GRIPPER_OPEN = [-0.3]
        self.GRIPPER_CLOSED = [0.3]
        self.GRIPPER_NEUTRAL = [0]
        # Connect to the right_arm move group
        self.arm = moveit_commander.MoveGroupCommander('arm')
        
        # Connect to the right_gripper move group
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
                
        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        self.arm.set_goal_joint_tolerance(0.001)
        self.gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        # arm.set_named_target('right_up')
        
        # Plan a trajectory to the goal configuration
        # traj = arm.plan()
        
        # Execute the planned trajectory
        # arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)


    def armCallback(self, msg):
        if msg.data == 'start_grasp':
            self.grasp_attempt()
        if msg.data == 'drop':
            self.place()
    def grasp_attempt(self):
        rospy.sleep(2)
        # Set the gripper target to neutal position using a joint value target
        self.gripper.set_joint_value_target(self.GRIPPER_OPEN)
        
        # Plan and execute the gripper motion
        self.gripper.go()
        rospy.sleep(1)
        
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [-0.06136, 0.358, 0.6034, 0.5471, 0.0]

        # Set the arm's goal configuration to the be the joint positions
        self.arm.set_joint_value_target(joint_positions)
                
        # Plan and execute the motion
        self.arm.go()
        rospy.sleep(1)
        
        # Save this configuration for later
        self.arm.remember_joint_values('saved_config', joint_positions)
        
        # Close the gripper as if picking something up
        self.gripper.set_joint_value_target(self.GRIPPER_CLOSED)
        self.gripper.go()
        rospy.sleep(1)
                
        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        self.arm.set_named_target('right_up')
        
        # Plan and execute the motion
        self.arm.go()
        rospy.sleep(1)
        rospy.loginfo("ARRIVING ")
        self.arm_pub.publish("picked")
    def place(self):        
        # Set the goal configuration to the named configuration saved earlier
        self.arm.set_named_target('saved_config')
        
        # Plan and execute the motion
        self.arm.go()
        rospy.sleep(1)
        
        # Open the gripper as if letting something go
        self.gripper.set_joint_value_target(self.GRIPPER_OPEN)
        self.gripper.go()
        rospy.sleep(1)
        
        # Return the arm to the named "resting" pose stored in the SRDF file
        self.arm.set_named_target('resting')
        self.arm.go()
        rospy.sleep(1)
        
        # Return the gripper target to neutral position
        self.gripper.set_joint_value_target(self.GRIPPER_NEUTRAL)
        self.gripper.go()
        rospy.sleep(1)

        self.arm_pub.publish("placed")
        rospy.sleep(4)
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

        

if __name__ == "__main__":
    try:
        MoveItDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
