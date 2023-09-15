#!conda run -n your_conda_env_name python

# Nesting State Machine
#!/usr/bin/env python

import rospy
import smach
import smach_ros
from utils import Speak, Listen

# define state Foo
class Initialisation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1','out0'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state INITIALISATION')
        if self.counter < 3:
            self.counter += 1
            print(f'{self.counter}')
            return 'out1'
        
        else:
            return 'out0'


def main():
    rospy.init_node('smach_trigger_nlp')

   # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALISING', Initialisation(), 
                               transitions={'out1':'SPEAKING', 
                                            'out0':'END'},
                            #    remapping={'foo_counter_in':'sm_counter', 
                            #               'foo_counter_out':'sm_counter'}
                                )
                                          
        
        smach.StateMachine.add('SPEAKING', Speak(text="Fuck you, lil bitch"), 
                               transitions={'out1':'LISTENING', 
                                            'out0':'END'},
                            #    remapping={'foo_counter_in':'sm_counter', 
                            #               'foo_counter_out':'sm_counter'}
                                )
        smach.StateMachine.add('LISTENING', Listen(), 
                               transitions={'out1':'END',
                                            'out0':'END'},
                            #    remapping={'bar_counter_in':'sm_counter'}
                                )


    # Execute SMACH plan
    outcome = sm.execute()
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()