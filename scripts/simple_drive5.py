#!/usr/bin/env python
import rospy, copy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
#from pimouse_ros.msg import SwitchValues
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
        rospy.Subscriber('/lightsensors', LightSensorValues, self.LightSensors)
#        rospy.Subscriber('/switchs', SwitchValues, self.Switchs)

    def LightSensors(self,messages):
        self.sensor_values = messages

#    def Switchs(self,messages):
#        self.switch_values = messages

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

        while not rospy.is_shutdown():
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

            self.cmd_vel.publish(self.data)
            rospy.loginfo(statemachine.state)
            rospy.loginfo(self.data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()
