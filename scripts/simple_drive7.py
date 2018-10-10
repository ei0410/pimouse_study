#!/usr/bin/env python
import rospy, copy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.msg import SwitchValues
from enum import Enum
import smach
import smach_ros

class INIT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_linear1', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('a ')

class Linear1(smach.State):
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


class StateMachine():
    class State(Enum):
        INIT    = 0
        LINEAR1 = 1
        TURN1   = 2
        LINEAR2 = 3
        TURN2   = 4
        LINEAR3 = 5
        TURN3   = 6
        LINEAR4 = 7
        TURN4   = 8
        STOP    = 9

    def __init__(self):
        self.state = self.State.INIT
        self.time_start = time.time()

    def update_state(self):
        standby_time = 1.0
        linear_time1 = standby_time + 2.0
        turn_time1   = linear_time1 + 1.0
        linear_time2 = turn_time1   + 2.0
        turn_time2   = linear_time2 + 1.0
        linear_time3 = turn_time2   + 2.0
        turn_time3   = linear_time3 + 1.0
        linear_time4 = turn_time3   + 2.0
        turn_time4   = linear_time4 + 1.0
        stop_time    = turn_time4   + 2.0

        elapsed_time = time.time() - self.time_start

        if elapsed_time < standby_time:
            self.state = self.State.INIT
        elif elapsed_time < linear_time1:
            self.state = self.State.LINEAR1
        elif elapsed_time < turn_time1:
            self.state = self.State.TURN1
        elif elapsed_time < linear_time2:
            self.state = self.State.LINEAR2
        elif elapsed_time < turn_time2:
            self.state = self.State.TURN2
        elif elapsed_time < linear_time3:
            self.state = self.State.LINEAR3
        elif elapsed_time < turn_time3:
            self.state = self.State.TURN3
        elif elapsed_time < linear_time4:
            self.state = self.State.LINEAR4
        elif elapsed_time < turn_time4:
            self.state = self.State.TURN4
        elif elapsed_time < stop_time:
            self.state = self.State.STOP
        else:
            self.state = self.State.STOP
    
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

    def up(self, vel):
        self.data.angular.z = 0.0
        self.data.linear.x = vel if self.sensor_values.sum_all < self.threshold else 0.0

    def down(self, vel):
        self.data.angular.z = 0.0
        self.data.linear.x = -vel if self.sensor_values.sum_all < self.threshold else 0.0

    def left(self, rot):
        self.data.linear.x = 0.0
        self.data.angular.z= rot if self.sensor_values.sum_all < self.threshold else 0.0

    def right(self, rot):
        self.data.linear.x = 0.0
        self.data.angular.z= -rot if self.sensor_values.sum_all < self.threshold else 0.0

    def stop(self):
        self.data.linear.x = 0.0
        self.data.linear.y = 0.0
        self.data.linear.z = 0.0
        self.data.angular.x = 0.0
        self.data.angular.y = 0.0
        self.data.angular.z = 0.0

    def run(self):
        rate = rospy.Rate(10)
        statemachine = StateMachine()

        vel_x = 0.2
        rot_z = 2.0

        #rospy.loginfo(self.switch_values)

        while not rospy.is_shutdown():
            """
            statemachine.update_state()

            if statemachine.state == statemachine.State.INIT:
                self.stop()
            elif statemachine.state == statemachine.State.LINEAR1:
                self.up(vel_x)
            elif statemachine.state == statemachine.State.TURN1:
                self.left(rot_z)
            elif statemachine.state == statemachine.State.LINEAR2:
                self.up(vel_x)
            elif statemachine.state == statemachine.State.TURN2:
                self.right(rot_z)
            elif statemachine.state == statemachine.State.LINEAR3:
                self.up(vel_x)
            elif statemachine.state == statemachine.State.TURN3:
                self.right(rot_z)
            elif statemachine.state == statemachine.State.LINEAR4:
                self.up(vel_x)
            elif statemachine.state == statemachine.State.TURN4:
                self.left(rot_z)
            elif statemachine.state == statemachine.State.STOP:
                self.stop()
            else :
                self.stop()
            """

            self.cmd_vel.publish(self.data)
            rospy.loginfo(statemachine.state)
            rospy.loginfo(self.data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.init_node('state_machine')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()

    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={'to_linear1':'LINEAR1','fail':'ERROR_STOP'})
        smach.StateMachine.add('LINEAR1', Linear1(), transitions={'to_turn1':'TURN1','fail':'ERROR_STOP'})
        smach.StateMachine.add('TURN1', Linear1(), transitions={'to_stop':'STOP','fail':'ERROR_STOP'})
        smach.StateMachine.add('STOP',   Stop(),   transitions={'to_stop':'STOP'})
        smach.StateMachine.add('ERROR_STOP',   Error_Stop(),   transitions={'to_stop':'FAIL'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    SimpleDrive().run()
