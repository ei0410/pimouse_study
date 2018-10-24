#!/usr/bin/env python
import rospy, copy
import time
import math
import smach
import smach_ros
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.msg import SwitchValues
from enum import Enum

class Stand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STAND')
        rospy.sleep(3)
        return 'to_up'

class Up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_left', 'to_right','to_stop'])
        self.count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state UP')
        rospy.sleep(1)
        self.count = self.count + 1
        if self.count > 5:
            return 'to_stop'
        elif self.count % 2 == 1:
            return 'to_left'
        else:
            return 'to_right'

class Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LEFT')
        rospy.sleep(1)
        return 'to_up'

class Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RIGHT')
        rospy.sleep(1)
        return 'to_up'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
        rospy.sleep(1)
        return 'to_finish'

class SimpleDrive():
    def __init__(self):
        self.data = Twist()
        self.threshold = 500
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sensor_values = LightSensorValues()
        self.switch_values = SwitchValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.LightSensors)
        rospy.Subscriber('/switchs', SwitchValues, self.Switchs)

    def LightSensors(self,messages):
        self.sensor_values = messages

    def Switchs(self,messages):
        self.switch_values = messages

    def linear(self, vel):
        self.data.linear.x = vel if self.sensor_values.sum_all < self.threshold else 0.0

    def turn(self, rot):
        self.data.angular.z = rot if self.sensor_values.sum_all < self.threshold else 0.0

    def run(self):
        rate = rospy.Rate(10)
        statemachine = StateMachine()

        vel_x = 0.2
        rot_z = 2.0

        while not rospy.is_shutdown():

            self.linear(vel_x)
            self.turn(rot_z)

            self.cmd_vel.publish(self.data)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()

    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])

    with sm:
        smach.StateMachine.add('STAND', Stand(), transitions={'to_up':'UP'})
        smach.StateMachine.add('UP', Up(), transitions={'to_left':'LEFT','to_right':'RIGHT','to_stop':'STOP'})
        smach.StateMachine.add('LEFT', Left(), transitions={'to_up':'UP'})
        smach.StateMachine.add('RIGHT', Right(), transitions={'to_up':'UP'})
        smach.StateMachine.add('STOP',   Stop(),   transitions={'to_finish':'SUCCESS'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    SimpleDrive().run()
