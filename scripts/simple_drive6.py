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
        INIT  = 0
        STAND = 1
        UP    = 2
        DOWN  = 3
        LEFT  = 4
        RIGHT = 5
        STOP  = 6

    def __init__(self):
        self.state = self.State.INIT
        self.time_start = time.time()

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time
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

        self.standby_time = 1.0
        self.up_time1 = self.standby_time + 2.0
        self.turn_time1   = self.up_time1 + 0.785
        self.up_time2 = self.turn_time1   + 2.0
        self.turn_time2   = self.up_time2 + 0.785
        self.up_time3 = self.turn_time2   + 2.0
        self.turn_time3   = self.up_time3 + 0.785
        self.up_time4 = self.turn_time3   + 2.0
        self.turn_time4   = self.up_time4 + 0.785
        self.stop_time    = self.turn_time4   + 2.0

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

        """
        if elapsed_time < self.standby_time:
            self.state = self.State.INIT
        elif elapsed_time < self.up_time1:
            self.state = self.State.UP
        elif elapsed_time < self.turn_time1:
            self.state = self.State.LEFT
        elif elapsed_time < self.up_time2:
            self.state = self.State.UP
        elif elapsed_time < self.turn_time2:
            self.state = self.State.RIGHT
        elif elapsed_time < self.up_time3:
            self.state = self.State.UP
        elif elapsed_time < self.turn_time3:
            self.state = self.State.RIGHT
        elif elapsed_time < self.up_time4:
            self.state = self.State.UP
        elif elapsed_time < self.turn_time4:
            self.state = self.State.LEFT
        elif elapsed_time < self.stop_time:
            self.state = self.State.STOP
        else:
            self.state = self.State.STOP
        """

        if (self.state == self.State.INIT) and (elapsed_time < self.standby_time):
            self.state = self.State.STAND
        elif ((self.state == self.State.STAND) and (self.standby_time < elapsed_time) and (elapsed_time < self.up_time1)) or ((self.state == self.State.LEFT) and (self.turn_time1 < elapsed_time) and (elapsed_time < self.up_time2)) or ((self.state == self.State.RIGHT) and (self.turn_time2 < elapsed_time) and (elapsed_time < self.up_time3)) or ((self.state == self.State.RIGHT) and (self.turn_time3 < elapsed_time) and (elapsed_time < self.up_time4)):
            self.state = self.State.UP
        elif ((self.state == self.State.UP) and (self.up_time1 < elapsed_time) and (elapsed_time < self.turn_time1)) or ((self.state == self.State.UP) and (self.up_time4 < elapsed_time) and (elapsed_time < self.turn_time4)):
            self.state = self.State.LEFT
        elif ((self.state == self.State.UP) and (self.up_time2 < elapsed_time) and (elapsed_time < self.turn_time2)) or ((self.state == self.State.UP) and (self.up_time3 < elapsed_time) and (elapsed_time < self.turn_time3)):
            self.state = self.State.RIGHT
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
            statemachine.update_state()

            if statemachine.state == statemachine.State.INIT:
                vel_x = 0.0
                rot_z = 0.0
            elif statemachine.state == statemachine.State.UP:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.LEFT:
                vel_x = 0.0
                rot_z = 2.0
            elif statemachine.state == statemachine.State.RIGHT:
                vel_x = 0.0
                rot_z = -2.0
            elif statemachine.state == statemachine.State.STOP:
                vel_x = 0.0
                rot_z = 0.0
            else :
                vel_x = 0.0
                rot_z = 0.0

            self.linear(vel_x)
            self.turn(rot_z)

            statemachine.odom_update(vel_x, rot_z)

            self.cmd_vel.publish(self.data)

            rospy.loginfo(str(statemachine.dt) + "," + str(statemachine.Lstep) + "," + str(statemachine.Rstep) + "," + str(statemachine.x) + "," + str(statemachine.y) + "," + str(statemachine.th) + "," + str(statemachine.state))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()
