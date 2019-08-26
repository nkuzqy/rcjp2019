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
        if msg.data == "Person Not Found":
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
        if "HANDSOME" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)
            self.soundhandle.say("I heard " + "who is the most handsome person in canada", volume=1)
            self.soundhandle.say("The answer is I heard that Justin Trudeau is very handsome", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")
        elif "RCMP" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "How big is the RCMP", volume=1)
            
            self.soundhandle.say("The answer is today, the RCMP has close to 30,000 members", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "MONTREAL" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what else is montreal called", volume=1)
            
            self.soundhandle.say("The answer is Montreal is often called the City of Saints or the City of a Hundred Bell Towers.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "LONGEST" in msg.data and "STREET" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the longest street in the world", volume=1)
            
            self.soundhandle.say("The answer is Yonge Street in Ontario is the longest street in the world.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "ICE" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "How many tons of ice are required to build The Hotel de Glace", volume=1)
            
            self.soundhandle.say("The answer is the Hotel de Glace requires about 400 tons of ice.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "LOCATED" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "Where is the hotel de glace located", volume=1)
            
            self.soundhandle.say("The answer is The Hotel de Glace is in Quebec.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "BEAR" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "what is the name of the bear cub exported from Canada to the London Zoo in 1915", volume=1)
            
            self.soundhandle.say("The answer is The bear cub was named Winnipeg, it inspired the stories of Winnie-the-Pooh.", volume= 1)
            self.s.play()
            rospy.loginfo("Start Speech Recognition")
            response = self.set_state(True)     
            self.count+=1
            if self.count == 3:
                rospy.loginfo("Stop Speech Recognition")
                response = self.set_state(False)
                self.soundhandle.say("basic functionality stop", volume=1)
                self.pub_to_control.publish("speech_end")

        elif "yonge" in msg.data:
            rospy.loginfo("Stop Speech Recognition")
            response = self.set_state(False)

            self.soundhandle.say("I heard " + "how long is yonge street in ontario", volume=1)
            
            self.soundhandle.say("The answer is Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border.", volume= 1)
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
