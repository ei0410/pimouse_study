#!/usr/bin/env python
import rospy, copy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class SimpleDrive():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def run(self):
        """
        rate = rospy.Rate(10)
        data = Twist()

        while not rospy.is_shutdown():
            data.linear.x = 0.2 if self.sensor_values.sum_all < 500 else 0.0
            self.cmd_vel.publish(data)
            rate.sleep()
        """
        rate = rospy.Rate(10)
        data = Twist()
        start = time.time()

        while not rospy.is_shutdown():
            data.linear.x = 0.1 if self.sensor_values.sum_all < 500 else 0.0
            elapsed_time = time.time() - start
            if elapsed_time > 3.0 :
                data.linear.x = 0.0
            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_drive')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    SimpleDrive().run()