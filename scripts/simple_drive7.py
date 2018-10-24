#!/usr/bin/env python

import rospy, copy
import time
import math
import smach
import smach_ros

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

standby_time = 1.0
linear_time1 = standby_time + 2.0
turn_time1   = linear_time1 + 0.785
linear_time2 = turn_time1   + 2.0
turn_time2   = linear_time2 + 0.785
linear_time3 = turn_time2   + 2.0
turn_time3   = linear_time3 + 0.785
linear_time4 = turn_time3   + 2.0
turn_time4   = linear_time4 + 0.785
stop_time    = turn_time4   + 2.0

time_start = time.time()

class Stand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up', 'to_stand'])
        self.elapsed_time = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STAND')
        self.elasped_time = time.time() - time_start

        if standby_time < self.elapsed_time:
            return 'to_up'
        else:
            return 'to_stand'

class Up(smach.State):
    def __init__(self):
        global elapsed_time
        smach.State.__init__(self, outcomes=['to_left', 'to_right','to_stop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP')

        elapsed_time = time.time() - time_start

        if elapsed_time < turn_time1:
            return 'to_left'
        else:
            pass

class Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LEFT')
        return 'to_up'

class Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RIGHT')
        return 'to_up'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
        time.sleep(3)
        return 'to_finish'

class State():
    def __init__(self):
        self.dt = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vth = 0.0

        self.forward_hz = 0
        self.rot_hz = 0

        self.Rstep = 0
        self.Lstep = 0

    def odom_update(self, vel, rot):
        self.cur_time = rospy.Time.now()
        self.dt = self.cur_time.to_sec() - self.last_time.to_sec()

        self.vx = vel
        self.vth = rot
        self.x += self.vx * math.cos(self.th) * self.dt
        self.y += self.vx * math.sin(self.th) * self.dt
        self.th += self.vth * self.dt

        self.forward_hz = 80000.0*vel/(9*math.pi)
        self.rot_hz = 400.0*rot/math.pi

        self.Lstep += int(round((self.forward_hz - self.rot_hz) * self.dt))
        self.Rstep += int(round((self.forward_hz + self.rot_hz) * self.dt))

        self.last_time = self.cur_time

    def update_state(self):

        elapsed_time = time.time() - self.time_start

        if elapsed_time < self.standby_time:
            self.state = self.State.INIT
        elif elapsed_time < self.linear_time1:
            self.state = self.State.LINEAR1
        elif elapsed_time < self.turn_time1:
            self.state = self.State.TURN1
        elif elapsed_time < self.linear_time2:
            self.state = self.State.LINEAR2
        elif elapsed_time < self.turn_time2:
            self.state = self.State.TURN2
        elif elapsed_time < self.linear_time3:
            self.state = self.State.LINEAR3
        elif elapsed_time < self.turn_time3:
            self.state = self.State.TURN3
        elif elapsed_time < self.linear_time4:
            self.state = self.State.LINEAR4
        elif elapsed_time < self.turn_time4:
            self.state = self.State.TURN4
        elif elapsed_time < self.stop_time:
            self.state = self.State.STOP
        else:
            self.state = self.State.STOP
    
class SimpleDrive():
    def __init__(self):
        self.data = Twist()
        self.threshold = 500
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.LightSensors)

    def LightSensors(self,messages):
        self.sensor_values = messages

    def linear(self, vel):
        self.data.linear.x = vel if self.sensor_values.sum_all < self.threshold else 0.0

    def turn(self, rot):
        self.data.angular.z = rot if self.sensor_values.sum_all < self.threshold else 0.0

    def run(self):
        rate = rospy.Rate(10)

        vel_x = 0.2
        rot_z = 2.0

        while not rospy.is_shutdown():
            self.linear(vel_x)
            self.turn(rot_z)
            self.cmd_vel.publish(self.data)
            rospy.loginfo(str(statemachine.dt) + "," + str(statemachine.Lstep) + "," + str(statemachine.Rstep) + "," + str(statemachine.x) + "," + str(statemachine.y) + "," + str(statemachine.th) + "," + str(statemachine.state))
            last_time = cur_time
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()

    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])

    with sm:
        smach.StateMachine.add('STAND', Stand(), transitions={'to_up':'UP','to_stand':'STAND'})
        smach.StateMachine.add('UP', Up(), transitions={'to_left':'LEFT','to_right':'RIGHT','to_stop':'STOP'})
        smach.StateMachine.add('LEFT', Left(), transitions={'to_up':'UP'})
        smach.StateMachine.add('RIGHT', Right(), transitions={'to_up':'UP'})
        smach.StateMachine.add('STOP',   Stop(),   transitions={'to_finish':'SUCCESS'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    SimpleDrive().run()
