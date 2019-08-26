#!/usr/bin/env python

"""
    talkback.py - Version 1.1 2013-12-20
    
    Use the sound_play client to say back what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from rcjp_speech_msgs.srv import SetRecState

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
        rospy.Subscriber('/lm_data', String, self.talkback)
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
        if msg.data == "Person Not Found!":
            self.start_rec = True
            # play signal sound
            self.soundhandle.say("Sorry I can not find you, please come to me and say the question after the signal sound", volume=1)
            rospy.sleep(4)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)

    def talkback(self, msg):
        # Print the recognized words on the screen
        self.count_talkback += 1
        rospy.loginfo(msg.data)
        if "TEAM" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)
            self.soundhandle.say("I heard " + "what is your team name", volume=1)
            self.soundhandle.say("Our team name is KameRider", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")
        elif "TODAY" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what day is today", volume=1)
            
            self.soundhandle.say("The answer is today is 16 August 2019", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "FROM" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "where are you from", volume=1)
            
            self.soundhandle.say("We are from Tianjin China.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "CAPITAL" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the capital of japan", volume=1)
            
            self.soundhandle.say("The answer is Tokyo.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "HEIGHT" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the height of mountain fuji", volume=1)
            
            self.soundhandle.say("The mountain is about 3776 meters", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "LONGEST" in msg.data or "RIVER" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the longest river in the world", volume=1)
            
            self.soundhandle.say("The answer is Nile river.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "AMERICAN" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "who is an american president", volume=1)
            
            self.soundhandle.say("The answer is Donald Trump.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "WHERE" in msg.data and "WE" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "where are we", volume=1)
            
            self.soundhandle.say("We are in Nagaoka.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "ANIMAL" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the heaviest animal in the world", volume=1)
            
            self.soundhandle.say("The heaviest animal is blue whale.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "COW" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "how many legs does the cow have", volume=1)
            
            self.soundhandle.say("A cow has 4 legs.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")


        
        elif self.count_talkback % 10 == 0:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("Sorry, please speak again", volume=1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")



    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
