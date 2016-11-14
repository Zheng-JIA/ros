#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

from numpy import *

LOOP_RATE = 30 # Hz

class Controller():
    def __init__(self, name, joy_topic, frame, worldFrame):
        self._frame = frame
        self._worldFrame = worldFrame

        self._buttons = None

        self._laststate = -1
        self._nextstate = 0

        self._state = [0,0,0,0,0,0]
        self._sensor = [0]
        self._home = [0,0,0]

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self._cmdvel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=True)

        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        rospy.Subscriber('/{}/state'.format(name), GenericLogData, self._stateChanged)
        rospy.loginfo("Controller initialized for {}".format(name))

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        starttime = None
        cmd = Twist()

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0

        while not rospy.is_shutdown():
            rate.sleep()

            # STATE ENTRY
            if self._laststate != self._nextstate:
                # EMERGENCY
                if self._nextstate == 0:
                    rospy.set_param("flightmode/posSet", 0)
                    self._update_params(["flightmode/posSet"])

                # LAND
                elif self._nextstate == 1:
                    self._home = [self._state[0], self._state[1], self._state[2]]
                    starttime = rospy.Time.now()

                # TAKEOFF
                elif self._nextstate == 2:
                    self._home = self._state[0:3]
                    starttime = rospy.Time.now()
                    print("takeoff from {}".format(self._home))
                    rospy.set_param("flightmode/posSet", 1)
                    self._update_params(["flightmode/posSet"])

                # CIRCLE
                elif self._nextstate == 3:
                    self._home = self._state[0:3]
                    starttime = rospy.Time.now()
                    rospy.set_param("flightmode/posSet", 1)
                    self._update_params(["flightmode/posSet"])

            self._laststate = self._nextstate

            # STATE ACTIONS

            # EMERGENCY
            if self._nextstate == 0:
                cmd.linear.x = 0
                cmd.linear.y = 0
                cmd.linear.z = 0

            # LAND
            elif self._nextstate == 1:
                dt = (rospy.Time.now() - starttime).to_sec()
                cmd.linear.x = self._home[0]
                cmd.linear.y = self._home[1]
                cmd.linear.z = 1000*max(0.25,self._home[2]-self._home[2]*dt/2)
                if dt > 2:
                    self._nextstate = 0 # enter emergency to turn off the motors.

            # TAKEOFF
            elif self._nextstate == 2:
                dt = (rospy.Time.now() - starttime).to_sec()
                cmd.linear.x = self._home[0]
                cmd.linear.y = self._home[1]
                cmd.linear.z = 0.5*min(1,1*dt/2)*1000

            # CIRCLE
            elif self._nextstate == 3:
                dt = (rospy.Time.now() - starttime).to_sec()
                cmd.linear.x = self._home[0] + 0.5*cos(2*pi*dt/5)
                cmd.linear.y = self._home[1] - 0.5*sin(2*pi*dt/5)
                cmd.linear.z = 1.0*1000

            self._cmdvel_pub.publish(cmd)


    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                # EMERGENCY
                if i == 0 and data.buttons[i] == 1:
                    self._nextstate = 0

                # LAND
                if i == 1 and data.buttons[i] == 1:
                    self._nextstate = 1

                # TAKEOFF TO HOVER
                if i == 2 and data.buttons[i] == 1:
                    self._nextstate = 2

                # CIRCLE
                if i == 3 and data.buttons[i] == 1:
                    self._nextstate = 3
                
                #  
                if i == 11 and data.buttons[i] == 1:
                    rospy.set_param("kalman/resetEstimation", data.buttons[i])
                    self._update_params(["kalman/resetEstimation"])
 

        self._buttons = data.buttons

    def _stateChanged(self, state):
        self._state = state.values

if __name__ == '__main__':
    rospy.init_node('crazyflie_controller', anonymous=True)

    name = rospy.get_namespace().strip('/')
    worldFrame = rospy.get_param("~worldFrame", "world")
    frame = rospy.get_param("~frame")
    joy_topic = rospy.get_param("~joy_topic", "joy")

    controller = Controller(name, joy_topic, frame, worldFrame)
    controller.run()
    rospy.spin()
