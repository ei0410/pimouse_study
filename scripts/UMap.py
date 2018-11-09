#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class UMap():
    def __init__(self):
        self.umap = rospy.Publisher('/umap', Twist, queue_size=1)

    def callback(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        while not rospy.is_shutdown():
            self.umap.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('UMap')
    UMap().run()
