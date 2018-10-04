#!/usr/bin/env python
import rospy, copy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.msg import SwitchValues
from enum import Enum

class state(Enum):
    LINEAR1 = 1
    TURN1   = 2
    LINEAR2 = 3
    TURN2   = 4
    LINEAR3 = 5
    TURN3   = 6
    LINEAR4 = 7
    TURN4   = 8

threshold = 500
data = Twist()

class SimpleDrive():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.sensor_values = LightSensorValues()
        self.switch_values = SwitchValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)
        rospy.Subscriber('/switchs', SwitchValues, self.callback2)

    def callback(self,messages):
        self.sensor_values = messages

    def callback2(self,messages):
        self.switch_values = messages

    def up(self, vel):
        data.angular.z = 0.0
        data.linear.x = vel if self.sensor_values.sum_all < threshold else 0.0

    def left(self, rot):
        data.linear.x = 0.0
        data.angular.z= rot if self.sensor_values.sum_all < threshold else 0.0

    def stop(self):
        data.linear.x = 0.0
        data.linear.y = 0.0
        data.linear.z = 0.0
        data.angular.x = 0.0
        data.angular.y = 0.0
        data.angular.z = 0.0

    def run(self):
        rate = rospy.Rate(10)
        start = time.time()

        vel_x = 0.2
        rot_z = 2.0

        linear_time1 = 2.0
        turn_time1 = linear_time1 + 1.0
        linear_time2 = turn_time1 + 2.0
        turn_time2 = linear_time2 + 1.0
        linear_time3 = turn_time2 + 2.0
        turn_time3 = linear_time3 + 1.0
        linear_time4 = turn_time3 + 2.0
        turn_time4 = linear_time4 + 1.0

        while not rospy.is_shutdown():
            elapsed_time = time.time() - start

            if elapsed_time < linear_time1:
                self.up(vel_x)
            elif elapsed_time < turn_time1:
                self.left(rot_z)
            elif elapsed_time < linear_time2:
                self.up(vel_x)
            elif elapsed_time < turn_time2:
                self.left(rot_z)
            elif elapsed_time < linear_time3:
                self.up(vel_x)
            elif elapsed_time < turn_time3:
                self.left(rot_z)
            elif elapsed_time < linear_time4:
                self.up(vel_x)
            else :
                self.stop()

            self.cmd_vel.publish(data)
            rospy.loginfo(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()
