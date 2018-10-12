#!/usr/bin/env python
#encoding: utf8
import rospy, copy
import time
import sys, rospy, math, tf
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.msg import SwitchValues
from pimouse_ros.msg import MotorFreqs
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry
from enum import Enum

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def set_power(self,onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en,'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self,left_hz,right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return 
        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf, open("/dev/rtmotor_raw_r0",'w') as rf: 
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)

    def callback_cmd_vel(self,message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        self.vth = message.angular.z

        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        dt = self.cur_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x, self.y, 0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time

        rospy.loginfo(self.x)

    def onoff_response(self,onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def callback_on(self,message): return self.onoff_response(True)
    def callback_off(self,message): return self.onoff_response(False)

    def callback_tm(self,message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev, 'w') as f:
                f.write("%d %d %d\n" % (message.left_hz, message.right_hz, message.duration_ms))
        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True

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
        motor = Motor()

        vel_x = 0.2
        rot_z = 2.0

        #rospy.loginfo(self.switch_values)

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
            motor.send_odom()
            rospy.loginfo("x:  " + str(motor.x))
            rospy.loginfo("y:  " + str(motor.y))
            rospy.loginfo("th: " + str(motor.th))
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
