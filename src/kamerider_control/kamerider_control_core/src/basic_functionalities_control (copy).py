#! /usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
import sys
import time
import wave
import datetime
import pyaudio
from kamerider_control_msgs.msg import Mission
from kamerider_control_msgs.msg import Result
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
UNSTART = 0
PROCESS = 1
FAILED  = 2
FINISH  = 3
class basic_func_control():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("basic_func control online~")
        # Type of task
        self.task_type = None
        self.pub_to_nav_topic_name = None
        self.pub_to_image_topic_name = None
        self.pub_to_arm_topic_name = None
        self.pub_to_speech_topic_name = None
        self.sub_nav_back_topic_name = None
        self.sub_image_back_topic_name = None
        self.sub_arm_back_topic_name = None
        self.sub_speech_back_topic_name = None

        # For pick and place
        self.drop_place = None

        # Mission state
        self._door = UNSTART
        self._room = UNSTART
        self._navigate = UNSTART
        self._mission = UNSTART
        self._arm = UNSTART
        self._speech = UNSTART

        self.get_params()

    def init_params(self):
        # Mission state
        self._door = UNSTART
        self._room = UNSTART
        self._navigate = UNSTART
        self._mission = UNSTART
        self._arm = UNSTART
        self._speech = UNSTART

    def get_params(self):
        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        self.sh.say("start control")
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        self.sub_image_back_topic_name = rospy.get_param("sub_image_back_topic_name", "/image_to_control")
        self.sub_arm_back_topic_name   = rospy.get_param("sub_arm_back_topic_name", "/arm_to_control")
        self.sub_nav_back_topic_name   = rospy.get_param("sub_nav_back_topic_name", "/nav_to_control")
        self.sub_speech_back_topic_name = rospy.get_param("sub_speech_back_topic_name", "/speech_to_control")

        self.pub_to_arm_topic_name   = rospy.get_param("pub_to_arm_topic_name", "/control_to_arm")
        self.pub_to_image_topic_name = rospy.get_param("pub_to_image_topic_name", "/control_to_image")
        self.pub_to_nav_topic_name   = rospy.get_param("pub_to_nav_topic_name", "/control_to_nav")
        self.pub_to_speech_topic_name = rospy.get_param("pub_to_speech_topic_name", "/control_to_speech")
        self.pub_to_moveit_topic_name = rospy.get_param("pub_to_moveit_topic_arm","/adjust_to_arm")

        self.sub_arm = rospy.Subscriber(self.sub_arm_back_topic_name, String, self.arm_callback)
        self.sub_image = rospy.Subscriber(self.sub_image_back_topic_name, Result, self.image_callback)
        self.sub_nav   = rospy.Subscriber(self.sub_nav_back_topic_name, Result, self.nav_callback)
        self.sub_speech   = rospy.Subscriber(self.sub_speech_back_topic_name, String, self.speech_callback)

        self.pub_arm   = rospy.Publisher(self.pub_to_arm_topic_name, Mission, queue_size=1)
        self.pub_image = rospy.Publisher(self.pub_to_image_topic_name, Mission, queue_size=1)
        self.pub_nav   = rospy.Publisher(self.pub_to_nav_topic_name, Mission, queue_size=1)
        self.pub_moveit = rospy.Publisher(self.pub_to_moveit_topic_name, String, queue_size=1)
        self.pub_speech   = rospy.Publisher(self.pub_to_speech_topic_name, Mission, queue_size=1)
        rospy.sleep(2)
        self.start_basic_func()
    
    def arm_callback(self, msg):
        if msg.data == "picked":
            self.navigate_to("drop_place")
            self.sh.say("go to drop place")
        if msg.data == "placed":
            self.navigate_to("avoid_that_start")
    
    def image_callback(self, msg):
        ####
        '''
        if msg.mission_type == "vision":
            if msg.result == "known":
                self.drop_place == "known"
            if msg.result == "unknown":
                self.drop_place == "trashbin"
        '''
        if msg.mission_type == "detect":
            if msg.result == "door_open":
                self._door = FINISH

        
    def nav_callback(self, msg):
        if msg.mission_type == "navigate":
            self._navigate = FINISH
            ######如果到达了开始avoid_that的地点
            '''
            if msg.result == "entrance":
                self.sh.say("basic functionalities start")
                self.navigate_to("pick_and_place_start")
            if msg.result == "pick_and_place_start":
                print("start pick and place")
                self.sh.say("start pick and place")
                msg_arm = String()
                msg_arm.data = "start_grasp"
                self.pub_moveit.publish(msg_arm)
            if msg.result == "drop_place":
                print("arrive drop place")
                msg_moveit = String()
                msg_moveit = "drop"
                self.pub_moveit.publish(msg_moveit)
            '''
            if msg.result == "avoid_that_start":
                print("start avoid that")
                self.sh.say("start avoid that")
                self.navigate_to("what_did_you_say_start")
            if msg.result == "what_did_you_say_start":
                print("start what did you say")
                self.sh.say("start what did you say")
                os.system("gnome-terminal -x bash -c 'roslaunch rchomeedu_speech sound_test.launch'")

    def speech_callback(self,msg):
        if msg.data == "speech_end":
            self.navigate_to("Exit")

    def navigate_to(self, location):
        msg = Mission()
        msg.mission_type = "navigate"
        msg.mission_name = location
        if self._navigate != PROCESS:
            self.pub_nav.publish(msg)
            self._navigate = PROCESS

    def start_basic_func(self):
        rospy.sleep(15)
        """
            '''
            door detect block
            uncomment if necessary
            '''
        """
        self.sh.say("i notice the door is closed")
        self.sh.say("please help me open the door after three second")
        #os.system("gnome-terminal -x bash -c 'rosrun inspection door_detect'")
        '''
        while (True):
            if self._door == FINISH:
                rospy.sleep(1)
                self.sh.say("Now the door is open")
                self.sh.say("thank you very much")
                break
        '''
        rospy.sleep(5)
        #os.system("rosnode kill /door_detect")
        #################################################
        ######### Maybe to the start place first ########
        #################################################
        self.navigate_to("avoid_taht_start")
    
    def cleanup(self):
        rospy.loginfo("basic_func control closed")

if __name__ == '__main__':
    rospy.init_node("basic_func_control", anonymous=False)
    ctrl = basic_func_control()
    rospy.spin() 
