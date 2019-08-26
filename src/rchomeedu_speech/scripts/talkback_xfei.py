#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    talkback.py - Version 1.1 2013-12-20
    
    Use the sound_play client to say back what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from rcjp_speech_msgs.srv import SetRecState
from answer_question import answer_question

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talkback')

        rospy.on_shutdown(self.cleanup)
        
        # Initial lm client
        rospy.wait_for_service("change_state")
        self.set_state = rospy.ServiceProxy("change_state", SetRecState)
        rospy.sleep(1)
        # Create the sound client object
        self.soundhandle = SoundClient(blocking=True)
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(4)
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        self.s = self.soundhandle.waveSound("question_start_signal.wav")
        print ("step 1")
        # Flag to start the node
        self.start_rec = False

        # Flag to stop the node
        self.stop_rec = False

        # Count the question answered
        self.count = 0

        #count the question heard
        self.count_talkback = 0
        # Subscribe human detection topic
        rospy.Subscriber("/API_detector/human_detection", String, self.start_speech_rec)

        # Subscribe to the recognizer output and set the callback function
        # 修改此处话题为讯飞识别节点的话题
        rospy.Subscriber('/baidu_output', String, self.talkback)
        rospy.loginfo("talk_back online......")
  
        #pub to control
        self.pub_to_control = rospy.Publisher("/speech_to_control", String, queue_size = 1)
    def start_speech_rec(self, msg):
        if msg.data == "Person Target Reached!":
            self.start_rec = True
            # play signal sound
            self.soundhandle.say("Now I find you, please come to me and say the question after the signal sound", volume=1)
            rospy.sleep(4)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)
        if msg.data == "Person Not Found":
            self.start_rec = True
            # play signal sound
            self.soundhandle.say("Sorry I can not find you, please come to me and say the question after the signal sound", volume=1)
            rospy.sleep(4)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)

    def talkback(self, msg):
        # 去除讯飞识别结果中的标点符号并分割为单个词语
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        if string[-1] in symbols:
            string = string[:-1]
        for part in string.lstrip().split(","):
            for word in part.split():
                output.append(word)
	    output = [item.lower() for item in output]
        

        # Print the recognized words on the screen
        self.count_talkback += 1
        rospy.loginfo(output)
        ans = "none match"
        ans = answer_question(output, self.count)

        # 因为讯飞只要有录音就一定会有识别结果的
        # 所以一定会进这个回调函数，3个问题每个3次机会
        # 只用监听 9次就可以，监听 9次的时候self.count数值为8
        # 所以在answer_question问题最后添加一个判断
        if ans == "none match":
            # 此时表示没有听到问题
            self.soundhandle.say("sorry i cant understand the question")
        elif ans == "end":
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(False)
            self.soundhandle.say("basic functionality stop", volume=1)
            self.pub_to_control("speech_end")
        else:
            self.soundhandle.say(ans)
        

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
