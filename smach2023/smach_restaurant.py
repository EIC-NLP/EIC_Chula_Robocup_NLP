# run with conda env: nlp
import roslib
import rospy
import smach
import smach_ros
import nlp_client
from ratfin import *
from person import Person
from utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,GetEntities)

class RestaurantDecision(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['checkout','order','cancel'],input_keys=['intent']
                             )
    def execute(self, userdata):
        if userdata.intent == 'restaurant_checkout':
            return 'checkout'
        elif userdata.intent == 'restaurant_order':
            return 'order'
        return 'cancel'

class DummyIdle(smach.State):
    '''
    DUMMY THICKKKKKKKKKKKK
    CANT BREATH WITH ALL THAT ASTHHH MAAAA
    '''
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1']
                             )

    def execute(self, userdata):
        return 'out1'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1','out0'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', DummyIdle(), 
                               transitions={'out1':'SPEAK_ASK'})
        
        smach.StateMachine.add('SPEAK_ASK', Speak(text="Hello bitch! How can I help you today?"), 
                               transitions={'out1':'LISTEN_INTENT'})
        
        smach.StateMachine.add('LISTEN_INTENT',GetEntities(intent=True,object=True,timeout=5),
                               transitions={'out1':'TASK_TRANSITION'},
                               remapping={'listen_intent':'intent'})
        
        smach.StateMachine.add('TASK_TRANSITION',RestaurantDecision(),
                               transitions={'cancel':'IDLE', 'checkout':'SPEAK_SUMMARIZE_ORDER', 'order':'PROCESS_ORDER'},
                               remapping={'intent':'intent'})
        
        smach.StateMachine.add('SPEAK_SUMMARIZE_ORDER',Speak(text='you ordered something!'),
                               transitions={'out1':'out0'})
        
        smach.StateMachine.add('PROCESS_ORDER',Speak(text='order something!'),
                               transitions={'out1':'out0'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()