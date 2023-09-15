
# MoveTo
# GraspObject
# PlaceObject 
# FollowPerson


# CarryMyLuggage 
# IdentifyObject
# Object Counting 
# Ibject analysis

import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *

def print_available_userdata(userdata):
    print(userdata)

# MoveTo state
class MoveTo(smach.State):
    """ 
    TemplateVersion 1.0.0 
    MoveTo state

    BASIC FUNCTION:
    Move from A to B with CV to find object/person
    1. [NLP] room, object, furniture, person, furniture_adjective,
        - [room, object, furniture, person] one must not be null
    2. [NAVIGATION] Move to the specified location
    3. [COMPUTER VISION] Detect the object/person. If not detected, repeat step 2
    3.1 [NLP] Announce "Unable to detect the object/person. Please try again" --> Loop
    4. [NLP] Announce "I have reached the location" --> out1

    """

    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['room','furniture','data3'],
                             output_keys=['data1','data3'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Set self.variables


    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(MoveTo): Executing..')

            # Userdata verification
            rospy.loginfo(f'(MoveTo): Checking userdata..')


            # Check if the userdata room, object,furniture is None
            if (userdata.room == "") and (userdata.furniture == "") and (userdata.object == "") and (userdata.person == ""):
                raise Exception(
                    "(MoveTo): Userdata room, object, furniture, and person; one must not be null")
            else:
                # if room is provided

                if (userdata.room != ""): 
                    # if furniture is provided
                    if userdata.furniture != "": 
                        # if furniture is found, move to the furniture (add nlp trying to find the furniture)
                        if userdata.person != "": # room+furniture+person
                            #TODO turn on CV to try to detect the person around the furniture in the room
                            return "out1"
                        elif userdata.object != "": # room+furniture+object
                            #TODO turn on CV to try to detect the object around the object in the room
                            return "out1"
                        else: # room+furniture
                            #TODO turn on CV to try to detect the furniture in a room
                            return "out1"
                    elif userdata.person != "": # room+person
                        #TODO turn on CV to try to detect the person and go to the person
                        return "out1"
                    elif userdata.object != "": # room+objet
                        #TODO turn on CV to try to detect the object around the room
                        return "out1"
                    # if furniture is not provided
                    else: # room
                        #* move to the center of the room
                        return "out1"

                # if room is not provided and furniture is provided
                elif (userdata.furniture != ""): 
                    if userdata.object != "": # furniture+object 
                        #TODO turn on CV to try to detect the object around the furniture (360 around furniture)
                        return "out1"
                    elif (userdata.person != ""): # furniture+person 
                        #TODO turn on CV to try to detect the person at the furniture (360 around furniture)
                        return "out1"
                    else: # furniture
                        #TODO turn on CV to try to detect the furniture in the entire arena
                        return "out1"

                # find the first person around the arena
                elif (userdata.person != ""):  # person 
                    #TODO turn on CV to try to detect the person in the entire arena
                    return "out1"
                else:
                    raise Exception("How the fuck did you get here!")






            # while (self.tries_counter < self.timeout_tries) or not self.timeout_bool:
            #     # Increment the counter
            #     self.tries_counter += 1


            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(MoveTo): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


