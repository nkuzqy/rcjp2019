#! /usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies. 
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
import sys
import time
import wave
import datetime
import pyaudio
import time
from kamerider_control_msgs.msg import Mission
from sound_play.libsoundplay import SoundClient
from kamerider_control_msgs.msg import Result
from kamerider_control_msgs.msg import MoveRobot

PI = 3.1415926
UNSTART = 0
PROCESS = 1
FINISH  = 2
FAILED  = 3

class Inspection(object):
    def __init__(self):
        # Publisher & Topics
        self.pub_nav = None
        self.pub_to_nav_topic_name = None

        # Subscriber & Topics
        self.sub_nav = None
        self.sub_nav_back_topic_name = None
        # Mission States
        self._restart    = 0
        self._navigate   = UNSTART
        self._door       = UNSTART

        # Set Params
        self.set_params()

        # Start Main Loop
        self.main_loop()
    
    def set_params(self):
        #Initialize sound client
        print ("Initialize Sound Client")
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")

        # set params through rosparam
        self.pub_to_nav_topic_name    = rospy.get_param("pub_to_nav_topic_name"  , "/control_to_nav")

        self.sub_nav_back_topic_name     = rospy.get_param("sub_nav_back_topic_name"  , "/nav_to_control")

        # initialize publishers & subscribers
        self.pub_nav    = rospy.Publisher(self.pub_to_nav_topic_name, Mission, queue_size=1)

        self.sub_nav    = rospy.Subscriber(self.sub_nav_back_topic_name, Result, self.nav_callback)
        self.sub_image  = rospy.Subscriber("/image_to_control", Result, self.image_callback)
    
    def nav_callback(self, msg):
        if msg.mission_type == "navigate":
            if msg.result == "success":
                self._navigate = FINISH
    
    def image_callback(self, msg):
        if msg.mission_type == "detect":
            if msg.result == "door_open":
                self._door = FINISH

    def navigate_to(self, target):
        msg = Mission()
        msg.mission_type = "navigate"
        msg.mission_name = target
        self.pub_nav.publish(msg)
        self._navigate = PROCESS

    def main_loop(self):
        rospy.sleep(20)
        # os.system("gnome-terminal -x bash -c 'source /home/rcjp_ws/devel/setup.zsh && rosrun inspection door_detect'")
        while (True):
            if self._door == FINISH:
                self.sh.say("Now the door is open")
                break
        
        self.navigate_to("Exit")
        while (True):
            if self._navigate == FINISH:
                self.sh.say("i have already arrived at the inspect point")
                self.sh.say("now i will start to introduce myself")
                self.sh.say("hello my name is jack")
                self.sh.say("i come from Team Kamerider")
                self.sh.say("nankai University")
                self.sh.say("china")
                self.sh.say("now i will navigate to the exit")
                break
        self.sh.say("ok i have finished the inspection")
        self.sh.say("Now i will leave the area")
        # self.pub_nav.publish("exit")
    
if __name__ == '__main__':
    rospy.init_node("inspection", anonymous=False)
    ctrl = Inspection()
    rospy.spin()
