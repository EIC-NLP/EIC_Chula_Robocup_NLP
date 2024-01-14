# run with conda env: nlp
import roslib
import rospy
import smach
import smach_ros
import nlp_client
from termcolor import colored
from person import Person
from utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)

# Task specific state
class IntroducePeople(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1', 'out0'],
                            input_keys=[''],
                            output_keys=[''])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')


        # Speak the text
        nlp_client.speak()

        return 'out1'




def main():
    speak_debug = False
    response_debug = False

    rospy.init_node('smach_task_')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    
    # Declear Variables for top-state
    

    with sm:

    
        
        smach.StateMachine.add('SPEAK_SCAN',
                            Speak(text=""""Welcome guest My name is Walkie I will be your receptionist today. 
                            Please stand still for a moment while I scan you."""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'DUMMY_CV',
                                            'out0': 'out0'}
                                        )
        
        
        

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
