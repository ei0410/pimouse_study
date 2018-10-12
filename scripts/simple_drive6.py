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

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vth = 0.0

        self.standby_time = 1.0
        self.linear_time1 = self.standby_time + 2.0
        self.turn_time1   = self.linear_time1 + 1.0
        self.linear_time2 = self.turn_time1   + 2.0
        self.turn_time2   = self.linear_time2 + 1.0
        self.linear_time3 = self.turn_time2   + 2.0
        self.turn_time3   = self.linear_time3 + 1.0
        self.linear_time4 = self.turn_time3   + 2.0
        self.turn_time4   = self.linear_time4 + 1.0
        self.stop_time    = self.turn_time4   + 2.0

    def odom_update(self, vel, rot):
        self.cur_time = rospy.Time.now()
        dt = self.cur_time.to_sec() - self.last_time.to_sec()

        self.vx = vel
        self.vth = rot
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt
        rospy.loginfo("dt:     " + str(dt))

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
        rot_z = 1.57

        #rospy.loginfo(self.switch_values)

        while not rospy.is_shutdown():
            statemachine.update_state()

            if statemachine.state == statemachine.State.INIT:
                vel_x = 0.0
                rot_z = 0.0
            elif statemachine.state == statemachine.State.LINEAR1:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.TURN1:
                vel_x = 0.0
                rot_z = 1.57
            elif statemachine.state == statemachine.State.LINEAR2:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.TURN2:
                vel_x = 0.0
                rot_z = -1.57
            elif statemachine.state == statemachine.State.LINEAR3:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.TURN3:
                vel_x = 0.0
                rot_z = -1.57
            elif statemachine.state == statemachine.State.LINEAR4:
                vel_x = 0.2
                rot_z = 0.0
            elif statemachine.state == statemachine.State.TURN4:
                vel_x = 0.0
                rot_z = 1.57
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
            rospy.loginfo("x:      " + str(statemachine.x))
            rospy.loginfo("y:      " + str(statemachine.y))
            rospy.loginfo("th:     " + str(statemachine.th))
            rospy.loginfo("status: " + str(statemachine.state))
            rospy.loginfo("\n" + str(self.data))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()