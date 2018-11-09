#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Stand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_forward'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STAND')
        rospy.sleep(5)
        return 'to_forward'

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_left', 'to_right','to_stop'])
        self.count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARD')
        #rospy.sleep(1)
        self.count = self.count + 1
        '''
        if self.count > 5:
            return 'to_stop'
        elif self.count % 2 == 1:
            return 'to_left'
        else:
            return 'to_right'
        '''
        if self.count == 1:
            rospy.sleep(3)
            return 'to_left'
        elif self.count == 2:
            rospy.sleep(5)
            return 'to_right'
        elif self.count == 3:
            rospy.sleep(3)
            return 'to_right'
        elif self.count == 4:
            rospy.sleep(5)
            return 'to_stop'

class Back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_left', 'to_right','to_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BACK')
        rospy.sleep(1)
        return 'to_stop'

class Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_forward','to_back'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LEFT')
        rospy.sleep(1.5)
        return 'to_forward'

class Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_forward','to_back'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RIGHT')
        rospy.sleep(1.5)
        return 'to_forward'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
        rospy.sleep(1)
        return 'to_finish'

if __name__ == '__main__':
    rospy.init_node('simple_drive')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])

    with sm:
        smach.StateMachine.add('STAND', Stand(), transitions={'to_forward':'FORWARD'})
        smach.StateMachine.add('FORWARD', Forward(), transitions={'to_left':'LEFT','to_right':'RIGHT','to_stop':'STOP'})
        smach.StateMachine.add('BACK', Back(), transitions={'to_left':'LEFT','to_right':'RIGHT','to_stop':'STOP'})
        smach.StateMachine.add('LEFT', Left(), transitions={'to_forward':'FORWARD','to_back':'BACK'})
        smach.StateMachine.add('RIGHT', Right(), transitions={'to_forward':'FORWARD','to_back':'BACK'})
        smach.StateMachine.add('STOP',   Stop(),   transitions={'to_finish':'SUCCESS'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
