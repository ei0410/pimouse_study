#!/usr/bin/env python

import rospy, copy
import time
import math

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.msg import SwitchValues
from enum import Enum

class StateMachine():
    class State(Enum):
        INIT    = 0
        LINEAR1 = 1
        STOP    = 2

    def __init__(self):
        self.state = self.State.INIT

        self.time_start = time.time()
        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

        self.dt = 0.0

        self.x = 0.0
        self.vx = 0.0

        self.forward_hz = 0

        self.standby_time = 1.0
        self.linear_time1 = self.standby_time + 10.0
        self.stop_time    = self.linear_time1 + 1.0

    def odom_update(self, vel):
        self.cur_time = rospy.Time.now()
        self.dt = self.cur_time.to_sec() - self.last_time.to_sec()

        self.vx = vel
        self.x += self.vx * self.dt

        self.forward_hz = 80000.0*vel/(9*math.pi)

        self.last_time = self.cur_time

    def update_state(self):

        elapsed_time = time.time() - self.time_start

        if elapsed_time < self.standby_time:
            self.state = self.State.INIT
        elif elapsed_time < self.linear_time1:
            self.state = self.State.LINEAR1
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

    def run(self):
        rate = rospy.Rate(10)
        statemachine = StateMachine()

        vel_x = 0.0 # define only

        while not rospy.is_shutdown():
            statemachine.update_state()

            if statemachine.state == statemachine.State.INIT:
                vel_x = 0.0
                rot_z = 0.0
            elif statemachine.state == statemachine.State.LINEAR1:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.STOP:
                vel_x = 0.0
                rot_z = 0.0
            else :
                vel_x = 0.0
                rot_z = 0.0

            self.linear(vel_x)
            statemachine.odom_update(vel_x)
            self.cmd_vel.publish(self.data)

            #rospy.loginfo(str(statemachine.dt) + "," + str(statemachine.x) + "," + str(statemachine.state) + "," + str(self.data.linear.x))
            rospy.loginfo('{:.3f}'.format(statemachine.dt) + "," + '{:.3f}'.format(statemachine.x) + "," + '{:.3f}'.format(self.data.linear.x) + "," + str(statemachine.state))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()
