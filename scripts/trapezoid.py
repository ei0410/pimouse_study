#!/usr/bin/env python
import rospy, copy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class Trapezoid():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        v_min = 0.2
        v_max = 1.98

        # If v_max is about 1.6 m/s, step motor will stop
        # v_max limit is 1.98 m/s

        accel = 0.02

        data.linear.x = 0.0

        while not rospy.is_shutdown():
            data.linear.x += accel

            if self.sensor_values.sum_all >= 50:
                data.linear.x = 0.0
            elif data.linear.x <= v_min:
                data.linear.x = v_min
            elif data.linear.x >= v_max:
                data.linear.x = v_max

            self.cmd_vel.publish(data)
            rospy.loginfo(data.linear.x)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trapezoid')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    Trapezoid().run()
