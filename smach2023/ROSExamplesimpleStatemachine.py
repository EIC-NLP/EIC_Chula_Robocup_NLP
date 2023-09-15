#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros

from std_msgs.msg import Empty

class setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])
    def execute(self, userdata):
        rospy.sleep(3.5)
        return 'setup_done'

class foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foo_succeeded', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                print("state foo is being preempted!!!")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)
        return 'foo_succeeded'

def child_term_cb(outcome_map):
    if outcome_map['FOO_CALC'] == 'foo_succeeded':
        return True
    elif outcome_map['FOO_RESET'] == 'invalid':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['FOO_RESET'] == 'invalid':
        return 'foo_reset'
    elif outcome_map['FOO_CALC'] == 'foo_succeeded':
        return 'foo_done'
    else:
        return 'foo_reset'

def monitor_cb(ud, msg):
    return False     

def main():
    rospy.init_node("preemption_example")

    foo_concurrence = smach.Concurrence(outcomes=['foo_done', 'foo_reset'],
                                        default_outcome='foo_done',
                                        child_termination_cb=child_term_cb,
                                        outcome_cb=out_cb)

    with foo_concurrence:
        smach.Concurrence.add('FOO_CALC', foo())
        smach.Concurrence.add('FOO_RESET', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('SETUP', setup(), transitions={'setup_done':'FOO'})
        smach.StateMachine.add('FOO', foo_concurrence, transitions={'foo_done':'BAR', 'foo_reset':'SETUP'}) 
        smach.StateMachine.add('BAR', foo(), transitions={'foo_succeeded':'FOO', 'preempted':'SETUP'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()