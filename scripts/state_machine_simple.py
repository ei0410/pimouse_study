#!/usr/bin/env python

import rospy
import smach
import smach_ros

#define state Linear
class Linear(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Linear')
        rospy.sleep(1)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop')
        rospy.sleep(1)
        return 'outcome2'

if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # create a smach state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # open the container
    with sm:
        smach.StateMachine.add('LINEAR', Linear(), transitions={'outcome1':'STOP','outcome2':'outcome4'})
        smach.StateMachine.add('STOP',   Stop(),   transitions={'outcome2':'LINEAR'})

    # create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # execute smach plan
    outcome = sm.execute()
    rospy.spin()
