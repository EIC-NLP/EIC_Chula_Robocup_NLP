
""" 

Guidelines for writing a state:

Consult example_smach_state.py for an ideal state
--------------------------------------------
--MUST HAVE: 

1. OUTCOME INFORMATION: Have to declear all
    outcomes=['out1','out2','loop','undo','fail'] # out2 is optional, a good practice should have loop & undo

2. INPUT/OUTPUT DATA: Have to declare all
    input_keys=['data1', 'data2'],
    output_keys=['data1', 'data2']

2. LOGGING: EACH STEP MUST HAVE LOGGING TO ROSPY
    rospy.loginfo('Executing state State1')
    rospy.loginfo(f'({name of class state}}): Executing..')
    rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')

3. REMAPPING: ADD TO LIST FOR CONSTRUCTOR 
    # Will be added to the state machine by inputs and outputs
    remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

4. EXCEPTION HANDLING: return loop if exception or undo

5. FIX DATATYPE: NO DYNAMIC TYPE
    # Fix datatype to string, int, float, bool, list, dict
    log : bool = False # have default value
    # Raise exceptions if any entity parameter is not of type bool
    if not isinstance(intent, bool):
        raise ValueError("Argument 'intent' must be of type bool")
    if None then raise exception for variable CANNOT BE NONE

--------------------------------------------
--OPTIONAL:  

"""



import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
import time
import os

from core_smach.nlp_emerStop import EmergencyStop

# Model Smach States
class ExampleState(smach.State):
    """ 
    TemplateVersion 1.1.0 
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
                             input_keys=['data1','data2','data3'],
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
            rospy.loginfo(f'(ExampleState): Executing..')

            # Userdata verification
            rospy.loginfo(f'(ExampleState): Checking userdata..')

            # Do something
            print(userdata)

            while (self.tries_counter < self.timeout_tries) or not self.timeout_bool:
                # Increment the counter
                self.tries_counter += 1



            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(ExampleState): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


def main():
    # Initialize the node
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.name = ""
    

    with sm:

        smach.StateMachine.add('CHECK_PEOPLE',
                            # Speak(text="Please stand still for a moment while I scan you."),
                            ExampleState(),
                            remapping={'people_index': 'people_index'},
                            transitions={'out1': 'SPEAK_SCAN',
                                        'out0': 'INTRODUCE_PEOPLE'}
                                        )
        
        
        
    

    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()

    
if __name__ == '__main__':
    main()

