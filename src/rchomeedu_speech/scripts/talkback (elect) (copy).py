#!/usr/bin/env python

"""
    talkback.py - Version 1.1 2013-12-20
    
    Use the sound_play client to say back what is heard by the pocketsphinx recognizer.
    
"""

import rospy, os, sys
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
#from rcjp_speech_msgs.srv import SetRecState

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talkback')

        rospy.on_shutdown(self.cleanup)
        
        # Initial lm client
        # rospy.wait_for_service("change_state")
        # self.set_state = rospy.ServiceProxy("change_state", SetRecState)
        rospy.sleep(10)
        # Create the sound client object
        self.soundhandle = SoundClient(blocking=True)
        # Wait a moment to let the client connect to the
        # sound_play server
        # rospy.sleep(2)
        # Make sure any lingering sound_play processes are stopped.
        # self.soundhandle.stopAll()

        self.soundhandle.say(" do you want cellphone", volume=1)

        rospy.sleep(5)
        self.soundhandle.say(" do you want keyboard", volume=1)
        # Flag to stop the node


        rospy.sleep(5)
        self.soundhandle.say(" do you want mouse", volume=1)
        # Flag to stop the node
        #count the question heard
        rospy.loginfo("talk_back online......")



    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
