#!/usr/bin/env python

"""
kill flask in background
kill -9 $(lsof -t -i:5000)
"""

# ros libraries
import roslib
import rospy
import smach
import smach_ros
import time
import tf
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Point, PoseStamped
from std_msgs.msg import String, Int16

# import computer vision and realsense
import cv2
import numpy as np
from util.realsense import Realsense

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

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util.custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError #
from sensor_msgs.msg import Image, CameraInfo #
from actionlib_msgs.msg import GoalStatus

# Utils function
from math import atan, pi
from util.environment_descriptor import EnvironmentDescriptor
from util.realsense import Realsense

import tf2_geometry_msgs

import math


class go_to_Navigation():
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def move(self,location):
        global ed
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = ed.get_robot_pose(location)
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        while True:
            result = self.move_base_client.get_state()
            rospy.loginfo("status {}".format(result))
            if result == GoalStatus.SUCCEEDED :
                return True
            else:
                return False
    
    def move_no_block(self, location):
        global ed
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = ed.get_robot_pose(location)
        self.move_base_client.send_goal(goal)


    def move_position(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)

        rospy.loginfo('{}'.format(position))

        if position:
            goal.target_pose.pose.position = position.position
            goal.target_pose.pose.position.z = 0
            goal.target_pose.pose.orientation = position.orientation
            

            self.move_base_client.send_goal(goal)

    def move_position_no_block(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)

        goal.target_pose.pose.position = position.position
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = position.orientation

        self.move_base_client.send_goal(goal)

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        
        
        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)


    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')

        global ed
        
        self.moving_msg = Twist()
        self.moving_msg.linear.x = 0.2

        global rs
        # Detect door opening
        x_pixel, y_pixel = 1280/2, 720/2
        frame_count = 0

        # just return
        return 'continue_Standby'

        while True:
            rospy.sleep(0.5)
            distance = rs.get_coordinate(x_pixel, y_pixel)[2]
            rospy.loginfo(distance)
            # filter lower distance
            if distance < 0.4:
                continue

            # check if have available frame consecutively
            if frame_count >= self.FRAME_COUNT_LIMIT:
                speak("door open")

                # move forward
                #Moving through entrance door
                start_time = time.time()
                while time.time() - start_time < 0:
                    rospy.loginfo("Moving Forward...")
                    self.moving_pub.publish(self.moving_msg)
                    rospy.sleep(0.1)

                rospy.loginfo("Stop Moving Forward")
                self.moving_msg.linear.x = 0
                self.moving_pub.publish(self.moving_msg)
                break

            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        
        return 'continue_Standby'

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

        def go_to_Navigation(location):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
            goal.target_pose.pose = ed.get_robot_pose(location)
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            while True:
                result = self.move_base_client.get_state()
                rospy.loginfo("status {}".format(result))
                if result == GoalStatus.SUCCEEDED :
                    return True
                else:
                    return False

        # navigate to standby position
        # speak("I'm moving to standby")

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

class Place_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Place_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])
        self.follow_cmd = String()
        self.follow_cmd_pub = rospy.Publisher("/human/follow_cmd",String,queue_size=1)


    def execute(self, userdata):
        rospy.loginfo('Executing state Place_luggage')
        speak("Please pick your bag from my arm")
        time.sleep(10)

        self.follow_cmd.data = "stop"
        self.follow_cmd_pub.publish(self.follow_cmd)


        return 'continue_standby'

class Check_position(smach.State):
    def __init__(self):

        global ed

        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Check_position')
        self.detect_radius = 0.5
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.loginfo("Exit Position: {} and {}".format(ed.get_robot_pose('exit'), ed.get_robot_pose('livingroom_exit')))


    def execute(self, userdata):
        

        rospy.loginfo('Executing state Check_position')
        global robot_inside, is_stop, target_lost, ed

        robot_inside = True

        exits = [ed.get_robot_pose("exit"), ed.get_robot_pose("livingroom_exit")]

        while True:
            
            pose = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now()-rospy.Duration.from_sec(1.0))

            distance = [math.sqrt((pose.transform.translation.x-exit_position.position.x)**2 + (pose.transform.translation.y-exit_position.position.y)**2) for exit_position in exits]
            # rospy.loginfo("Distance is {} m".format(distance))
            # rospy.loginfo("Base footprint pose is  {}".format(pose.transform.translation))
            if distance[0]<self.detect_radius or distance[1] < self.detect_radius:

                if robot_inside:
                    if distance[0]<self.detect_radius:
                        speak("Exiting the office room")
                    else:
                        speak("Exiting the living room")

                robot_inside = False
                rospy.loginfo("Robot outside exit, Switch to non-move_base person follower")


                       
            if is_stop or target_lost:
                return 'continue_stop'
            

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


class Follow_person(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Follow_person')

        self.cancel = Twist()
        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)

        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        # follow with whole body
        self.follow_cmd = String()
        self.follow_cmd_pub = rospy.Publisher("/human/follow_cmd",String,queue_size=1)

        self.follow_cmd.data = "follow"
        self.is_cancelled = False

        # realsense follow
        self.realsense_follow_cmd = String()
        self.realsense_follow_cmd_pub = rospy.Publisher("/human/realsense_follow_cmd", String, queue_size=1)

    def execute(self, userdata):

        rospy.loginfo('Executing state Follow_person')
        global target_lost, is_stop, goal_pose, robot_inside, navigation

        goal_send_interval = 1.5 # send goal at least every 5 seconds or wait until previous goal.
        start_time = 0

        self.is_cancelled = False
        
        while True:

            if robot_inside :
                
                self.follow_cmd.data = "stop"
                self.follow_cmd_pub.publish(self.follow_cmd)

                self.realsense_follow_cmd_pub.publish("follow")

                try:

                    if time.time() - start_time > goal_send_interval:

                        navigation.move_position_no_block(goal_pose)
                        start_time = time.time()

                        # rospy.loginfo("Sending new goal: X,Y,Z is {}, {}, {}".format(pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z))

                        if  is_stop:
                            navigation.move_base_client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            self.realsense_follow_cmd_pub.publish("stop")
                            speak("I'm stopped")
                            return "continue_stop"

                        elif target_lost:
                            navigation.move_base_client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            self.realsense_follow_cmd_pub.publish("stop")
                            speak("I have lost you, where are you my friend.")
                            return "continue_stop"
                    else:
                        
                        
                        # wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                        if target_lost:
                            navigation.move_base_client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            speak("I have lost you, where are you my friend.")
                            self.realsense_follow_cmd_pub.publish("stop")
                            return "continue_stop"


                except Exception as e:

                    # rospy.loginfo(e)

                    if  is_stop:
                        navigation.move_base_client.cancel_goal()
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        self.realsense_follow_cmd_pub.publish("stop")
                        return "continue_stop"
                    
                    if target_lost:
                        navigation.move_base_client.cancel_goal()
                        self.stop_pub.publish(self.cancel)
                        speak("I have lost you, where are you my friend.")
                        self.realsense_follow_cmd_pub.publish("stop")
                        return "continue_stop"
            else:
                
                self.realsense_follow_cmd_pub.publish("stop")
                try:
                    if not self.is_cancelled:
                        navigation.move_base_client.cancel_goal()
                        self.follow_cmd_pub.publish("follow")
                        self.is_cancelled = True
                        
                    if  is_stop:
                        self.follow_cmd_pub.publish("stop")
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        return "continue_stop"

                    elif target_lost:
                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I lost you, where are you my friend.")
                        return "continue_stop"

                except Exception as e:

                    if  is_stop:

                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        return "continue_stop"

                    elif target_lost:
                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I lost you, where are you my friend.")
                        return "continue_stop"
                

class Get_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Get_bounding_box')

        # image
        self.x_pixel = None
        self.y_pixel = None
        self.bridge = CvBridge()

        # condition variable
        self.lost_frame = 0
        self.lost_threshold=30

        self.rel_pub  = rospy.Publisher("/human/rel_coor", Point, queue_size=1)
        self.abs_pub  = rospy.Publisher("/human/abs_coor", Point, queue_size=1)

        self.rel_point = Point()
        self.abs_point = Point()

        # image publisher for visualize
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

        #pose and last pose for following person
        self.pose = None
        self.last_pose = None

        #Tranform manager
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


    def execute(self, userdata):

        def transform_pose(input_pose, from_frame, to_frame):
            # **Assuming /tf2 topic is being broadcasted
            pose_stamped = PoseStamped()
            pose_stamped.pose = input_pose
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rospy.Time.now()
            output_pose_stamped = None
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                while not self.tf_buffer.can_transform:
                    rospy.loginfo("Cannot transform from {} to {}".format(from_frame, to_frame))
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))

                return output_pose_stamped.pose


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

        def detect(frame):
            global rs, target_lost, person_id, last_pose, personTrack
            
            self.pose = self.last_pose
            lost = True
            
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # check if there aren't any person when a target is already established, increase lost_frame
            if (len(result["result"]) == 0) and (person_id!=-1):

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                self.lost_frame += 1
                rospy.loginfo("Lost target for {} consecutive frames".format(self.lost_frame))

                if self.lost_frame >= self.lost_threshold:
                    target_lost = True

                return 

            # not found person yet
            if person_id == -1:

                lost = False

                center_pixel_list = []
                for track in result["result"]:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array

                    w = abs(track[2][0]-track[2][2])
                    h = abs(track[2][1]-track[2][3])
                    if (w*h < 200*100) or (self.x_pixel==1280) or (self.y_pixel == 720):
                        continue
                    
                    center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)
                

                if len(center_pixel_list)!=0:   
                    # find closest person
                    person_id = min(center_pixel_list, key=lambda x: x[2])[3] # get person id with min depth
                    speak("Found target")
                    rospy.loginfo("Found target person: target ID {}".format(person_id))

            else:


                for track in result["result"]:
                    # track : [id, class_name, [x1,y1,x2,y2]]
                    # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))
                    if track[0] == person_id:

                        self.x_pixel = int((track[2][0]+track[2][2])/2)
                        self.y_pixel = int((track[2][1]+track[2][3])/2)
                        self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)

                        #Check the size of bouding box to prevent any mismatching person frame
                        w = abs(track[2][0]-track[2][2])
                        h = abs(track[2][1]-track[2][3])
                        if (w*h < 200*100) or (self.x_pixel==1280) or (self.y_pixel == 720):
                            rospy.loginfo("Bounding box is to small")
                            continue  

                        #If the same target ID is found and size of bounding box is OK, then it is not lost
                        lost = False
                        self.lost_frame = 0
                
                        # visualize purpose
                        frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                        frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                        frame = cv2.putText(frame, str(person_id), rs.rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # check if person tracker can find any person
            if lost:

                self.lost_frame += 1

                rospy.loginfo("Lost target for {} consecutive frames".format(self.lost_frame))

                if self.lost_frame >= self.lost_threshold:
                    target_lost = True
                    # person_id = -1
            
            else:
                # 3d pose
                x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            
                self.abs_point.x = x_coord
                self.abs_point.y = y_coord
                self.abs_point.z = z_coord

                self.rel_point.x = self.x_pixel - frame.shape[1]/2
                self.rel_point.y = self.y_pixel - frame.shape[0]/2

                self.abs_pub.publish(self.abs_point)
                self.rel_pub.publish(self.rel_point)

                if 0.5 < z_coord < 8 and (500<self.x_pixel<700):
                    # rospy.loginfo("Target is at: ({}, {})".format(self.x_pixel, self.y_pixel))
                    # rospy.loginfo("Depth is {}".format(depth))
                    # rospy.loginfo("Detected at (x,y,z): {}".format([r/1000 for r in result]))
                    # rospy.sleep(0.01)

                    # camera frame for tf x is point toward and y is point left.
                    x = z_coord - 0.5 # set the goal point to be 1 meter away from person
                    y = -x_coord
                    z = -y_coord

                    #Save posestamped in pose format and tranform it to base footprint for yaw calculation
                    self.pose = PoseStamped()
                    self.pose.header.frame_id = 'realsense_pitch'
                    self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z = x, y, z 
                    self.pose = transform_pose(self.pose.pose, from_frame= self.pose.header.frame_id, to_frame='base_footprint')

                    #Calculate yaw angle
                    delta_x = self.pose.position.x
                    delta_y = self.pose.position.y
                    yaw = math.atan(delta_y/delta_x) # yaw

                    #Save and transform to map frame
                    self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, yaw)
                    self.pose = transform_pose(self.pose, from_frame= 'base_footprint', to_frame='map')

            self.last_pose = self.pose

        rospy.loginfo('Executing state Get_bounding_box')
        global target_lost, is_stop, person_id, goal_pose

        # reset variable
        self.x_pixel = None
        self.y_pixel = None
        self.pose = None
        self.last_pose = None
        person_id = -1
        self.lost_frame = 0
        rospy.sleep(0.5)

        # reset
        rs.reset()
        while True:

            detect(rs.get_image())
            rospy.sleep(0.01)

            if self.pose:
                goal_pose = self.pose

            if  target_lost == True:
                return 'continue_stop'
            
            elif is_stop == True:
                return 'continue_stop'


if __name__ == '__main__':
    # initiate ROS node
    rospy.init_node('carry_my_luggage')
    # initiate the global variable
    target_lost = False
    is_stop = False
    robot_inside = True
    person_id = None
    stop_rotate = False
    goal_pose = None

    ed = EnvironmentDescriptor('/home/eic/ros/smach/smach_task/config/fur_data_onsite.yaml')
    ed.read_yaml()

    navigation = go_to_Navigation()

    # person tracker
    host = "0.0.0.0"
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    realsense_pitch_reset_pub = rospy.Publisher('/realsense_pitch_absolute_command',Int16, queue_size=1)
    realsense_yaw_reset_pub = rospy.Publisher('/realsense_yaw_absolute_command',Int16, queue_size=1)

    realsense_pitch_reset_pub.publish(0)
    realsense_yaw_reset_pub.publish(0)



    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Start_signal',Start_signal(),
                                transitions={'continue_Standby':'Standby'})

        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_follow':'FOLLOW'})
        
        
        smach.StateMachine.add('Ask_if_arrived',Ask_if_arrived(),
                                transitions={'continue_standby':'Standby',
                                             'continue_place_luggage':'Place_luggage'})

        smach.StateMachine.add('Place_luggage',Place_luggage(),
                                transitions={'continue_standby':'Standby'})
        
        # Create sub smach state machine
        sm_follow = smach.Concurrence(outcomes=['Stop'], default_outcome = 'Stop')      
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
            smach.Concurrence.add('Check_position',Check_position())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Stop':'Ask_if_arrived'})
    

        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()