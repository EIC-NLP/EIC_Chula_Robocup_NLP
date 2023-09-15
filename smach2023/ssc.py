
# for developement

import smach
from nlp_client import *
from ratfin import *
import sys


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

# add the path for live ROBOCUP2023-NLP
sys.path.append('/home/robocup2023-nlp/ROBOCUP2023-NLP')
import smach
import rospy
# Define the states
class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success', 'failure', 'loop'],
                             input_keys=['data1', 'data2'],
                             output_keys=['data1', 'data2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State1')
        return 'success'

class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'loop'])

    def execute(self, userdata):
        # log rospy
        rospy.loginfo('Executing state State2')
        return 'success'



def generate_state_dict_list(state_sequence):
    state_dict_list = []
    
    for i in range(len(state_sequence)):
        current_state = state_sequence[i]
        state_obj = state_mapping[current_state]
        current_state_with_suffix = current_state + "_" + str(i + 1)  # appending the suffix

        # Define remappings here
        remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

        if i == len(state_sequence) - 1:  # Last state
            transitions = {'success': 'final_outcome', 
                           'failure': state_sequence[i - 1] + "_" + str(i) if i > 0 else 'final_outcome', 
                           'loop': current_state_with_suffix}
        else:
            transitions = {'success': state_sequence[i + 1] + "_" + str(i + 2), 
                           'failure': state_sequence[i - 1] + "_" + str(i) if i > 0 else state_sequence[i + 1] + "_" + str(i + 2), 
                           'loop': current_state_with_suffix}

        state_dict_list.append({
            'state_name': current_state_with_suffix,
            'state_obj': state_obj,
            'transitions': transitions,
            'remappings': remappings  # Added remappings here
        })

    return state_dict_list




def construct_smach_state_machine(states_dict_list):
    sm = smach.StateMachine(outcomes=['final_outcome'])

    with sm:
        for state_dict in states_dict_list:
            smach.StateMachine.add(state_dict['state_name'], 
                                   state_dict['state_obj'],
                                   transitions=state_dict['transitions'])

    return sm


# Mapping of state names to classes
state_mapping = {
    'State1': State1(),
    'State2': State2(),
    # Add more states here...
}

# Generate state dict list
state_sequence = ['State1', 'State2','State1']

state_dict_list = generate_state_dict_list(state_sequence)
print(state_dict_list)

# Construct state machine
sm = construct_smach_state_machine(state_dict_list)
sm.execute()
