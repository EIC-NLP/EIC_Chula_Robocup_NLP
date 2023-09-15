# ros libraries = 0
roslib = 0
rospy = 0
smach = 0
smach_ros = 0
time = 0
tf = 0
tf2_ros = 0
tf2_msgs = 0
geometry_msgs =0
std_msgs =0
Image =0
MoveBaseAction= 0
def CvBridge():
    pass
# import computer vision and realsense
import cv2
import numpy as np

# import for speech-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

#Bring in the simple action client
import actionlib

class Standby(smach.State):

    
    def __init__(self):
        rospy.loginfo('Initiating state Standby')
        smach.State.__init__(self, outcomes=['continue_follow'])
        self.has_bag = False
        # image publisher for visualize
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        self.bridge = CvBridge()
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
    def execute(self, userdata):
        global stt, is_stop, target_lost, rs
        is_stop = False
        target_lost = False
        rospy.loginfo('Executing state Standby')
        start_time = 0


        # standby = go_to_Navigation('carry_my_luggage_standby')
        # if standby:
        #     rospy.loginfo('Walky stand by, Ready for order')
        # else:
        #     rospy.logerr('Navigation failed')

        speak("Please stand around 1 meter away from me")
        time.sleep(0.5)
        speak("when you are ready, please say follow me after the beep")
        start_time = 0
        while True:
            
            # for visualize
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(rs.get_image(), "bgr8"))

            if stt.body is not None:
                print(stt.body)
                if stt.body["intent"] == "follow_people": # waiting for "follow me" command
                    stt.clear()
                    speak("I'm following you")
                    time.sleep(0.5)
                    speak("Please say hey walkie, and after the signal tell me to stop")
                    # Put bag hanger position code in here TODO
                    self.has_bag = True
                    return "continue_follow"
                else:
                    stt.clear()
                    speak("pardon?")
                    stt.listen()
                    start_time = time.time()

            if time.time() - start_time > 7:
                if not self.has_bag:
                    speak("Please put your bag on my arm")
                speak("when you are ready, please say follow me after the beep")
                start_time = time.time()
                stt.listen()
                

            time.sleep(0.01)


class Ask_if_arrived(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Ask_if_arrived')
        smach.State.__init__(self, outcomes=['continue_standby','continue_place_luggage'])

        

    def execute(self, userdata):
        global stt
        rospy.loginfo('Executing state Ask_if_arrived')
        speak("Are we arrived?")
        speak("the Please say yes or no after the signal")

        stt.clear()
        stt.listen()
        start_time = time.time()
        while True:
            
            if stt.body:
                rospy.loginfo(stt.body["intent"])

                if stt.body["intent"] == "affirm": # waiting for "follow me" command
                    stt.clear()
                    return "continue_place_luggage"

                elif stt.body["intent"] == "deny": # waiting for "carry my luggage" command
                    stt.clear()
                    speak("Please say follow me if you want me to follow you again")
                    return "continue_standby"

                else:
                    speak("Please say yes or no")
                    stt.clear()
                    stt.listen()

            if time.time() - start_time > 7:
                speak("the Please say yes or no after the signal")
                start_time = time.time()
                stt.listen()
            time.sleep(0.01)

class Stop_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Stop_command')

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        global target_lost, is_stop, stt
        # Wait for "stop" command or target lost
        while True:
            if stt.body is not None:
                if stt.body["intent"] == "stop":
                    is_stop = True
                    stt.clear()
                    return "continue_stop"

            if target_lost:
                return "continue_stop"
            time.sleep(0.01)