
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
# Import libraries
import rospy
import smach
import threading
from ratfin import *
import time
import nlp_client
from datetime import datetime

from core_smach.nlp_emerStop import EmergencyStop

# Lookup for time
hours = ["twelve", "one", "two", "three", "four", "five",
             "six", "seven", "eight", "nine", "ten", "eleven", "twelve"]
tens = ["", "", "twenty", "thirty", "forty", "fifty"]
ones = ["", "one", "two", "three", "four", "five",
        "six", "seven", "eight", "nine"]
teens = ["ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen", 
            "sixteen", "seventeen", "eighteen", "nineteen"]

# TellTime Smach States
class TellTime(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """

    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0, # 0 means infinite tries
                 time: datetime = datetime.now()
                 ):
        """ 
         To test custom time use 
        >>> test_time = datetime.strptime('12:59', '%H:%M')
        >>> print(tell_time(test_time)) """
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if not isinstance(time, datetime):
            raise ValueError("Argument 'time' must be of type datetime")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['time_str'],
                             output_keys=['time_str'])

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
            rospy.loginfo(f'(TellTime): Executing..')

            # Userdata verification
            rospy.loginfo(f'(TellTime): Checking userdata..')

            # Do something
            print(userdata)
            
            while (self.tries_counter < self.timeout_tries) or not self.timeout_bool:
                # Increment the counter
                self.tries_counter += 1
                hour = int(time.strftime('%I')) # 12-hour format
                minute = int(time.strftime('%M'))
                
                # Get the hour in words
                hour_word = hours[hour]

                # If minute is 0
                if minute == 0:
                    to_speak =  f"It's {hour_word} o'clock."
                else:
                    # If minute is a 'teen'
                    if 10 <= minute < 20:
                        minute_word = teens[minute-10]
                    
                    # If minute is non-zero but not a 'teen'
                    else:
                        minute_word = tens[minute//10]
                        if minute % 10 > 0:
                            minute_word += " " + ones[minute % 10]
                    to_speak =  f"It's {hour_word} {minute_word}."
                
                # Connect to nlp and speak the time
                nlp_client.speak(to_speak)
                



            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(ExampleState): No attribute detected in the timeout period")
            userdata.to_speak = ""
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


def main():
    # Initialize the node 
    rospy.init_node("smach_task_receptionist")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    
    # Declear Variables for top-state
    sm.userdata.time_str = ""

    with sm:

        smach.StateMachine.add('TELL_TIME',
                            # Speak(text="Please stand still for a moment while I scan you."),
                            TellTime(),
                            remapping={'time_str': 'time_str'},
                            transitions={'out1': 'out0',
                                        'out0': 'out0'}
                                        )

    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Emergency stop using "Walkie Freeze"

    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()

    
if __name__ == '__main__':
    main()

