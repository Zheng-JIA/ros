#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData, FullControl
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
import numpy as np

LOOP_RATE = 30# Hz

class Controller():
    def __init__(self, name, joy_topic, frame, worldFrame):
        self._frame = frame
        self._worldFrame = worldFrame

        self._buttons = None

        self._laststate = -1
        self._nextstate = 0

        self._state = [0,0,0,0,0,0]
        self._stateVel = [0,0,0]
        self._sensor = [0]
        self._home = [0,0,0]

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self._cmdvel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=True)
        self._cmdstate_led_pub = rospy.Publisher("cmd_state_led", Float32, queue_size=10)
        self._cmdfullcontrol_pub = rospy.Publisher("full_control", FullControl, queue_size=1)

        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        enable_logging = rospy.get_param("crazyflie_add/enable_logging")
        if enable_logging == 1:
            rospy.Subscriber('/{}/state'.format(name), GenericLogData, self._stateChanged, enable_logging)
        else:
            rospy.Subscriber('/{}/external_position'.format(name), PointStamped, self._stateChanged, enable_logging)
        rospy.Subscriber('/{}/stateVel'.format(name), GenericLogData, self._stateVelChanged)

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        starttime = None
        cmd = Twist()
        cmd_fullcontrol = FullControl()
        toggleState = 1

        cmd_fullcontrol.x = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        cmd_fullcontrol.y = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        cmd_fullcontrol.z = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        cmd_fullcontrol.yaw = np.array([0.0, 0.0], dtype=np.float32)

        while not rospy.is_shutdown():
            rate.sleep()

            # STATE ENTRY
            if self._laststate != self._nextstate:
                # EMERGENCY
                if self._nextstate == 1:
                    print("enter emergency")

                # TAKEOFF
                elif self._nextstate == 2:
                    starttime = rospy.Time.now()
                    self._home = self._state[0:3]
                    print("takeoff from {}".format(self._home))

                # LANDING
                elif self._nextstate == 3:
                    starttime = rospy.Time.now()
                    self._home = self._state[0:3]
                    print("landing...")

                # FIGURE 8 TRAJECTORY
                elif self._nextstate == 5:
                    starttime = rospy.Time.now()
                    self._home = self._state[0:3]
                    print("Track figure 8 trajectory")

                # SPIN
                elif self._nextstate == 6:
                    starttime = rospy.Time.now()
                    self._home = self._state[0:3]
                    print("spin")
 
                # DISTRIBUTED CONTROL move to a point in space
                elif self._nextstate == 7:
                    starttime = rospy.Time.now()
                    self._home = self._state[0:3]


            self._laststate = self._nextstate

            # STATE ACTIONS

            # EMERGENC
            if self._nextstate == 1:
                cmd_fullcontrol.enable = 0

            # TAKEOFF
            elif self._nextstate == 2:
                dt = (rospy.Time.now()-starttime).to_sec()
                cmd_fullcontrol.enable = 1
                cmd_fullcontrol.x[0] = self._home[0]
                cmd_fullcontrol.y[0] = self._home[1]
                cmd_fullcontrol.z[0] = self._home[2] + min(0.5, 0.5*dt)

            # LANDING
            elif self._nextstate == 3: 
                dt = (rospy.Time.now()-starttime).to_sec()
                cmd_fullcontrol.x[0] = self._home[0]
                cmd_fullcontrol.y[0] = self._home[1]
                cmd_fullcontrol.z[0] = max(0.25, self._home[2] - self._home[2]*dt/2)
                if dt > 2:
                    self._nextstate = 1

            # FIGURE 8 TRAJECTORY
            elif self._nextstate == 5:
                T = 16
                dt = (rospy.Time.now()-starttime).to_sec()
                cmd_fullcontrol.x[0] = self._home[0] - np.sin(dt*2*np.pi/T)
                cmd_fullcontrol.y[0] = self._home[1] + np.sin(  dt*2*np.pi/(T/2)    )
                cmd_fullcontrol.z[0] = 0.5
                # velocity
                cmd_fullcontrol.x[1] = - (2*np.pi/T)*np.cos(dt*2*np.pi/T)
                cmd_fullcontrol.y[1] = (2*np.pi/T)*np.cos(  dt*2*np.pi/(T/2)  )
                # acceleration
                cmd_fullcontrol.x[2] = (2*np.pi/T)**2*np.sin(dt*2*np.pi/T)
                cmd_fullcontrol.y[2] = - (2*np.pi/T)**2*np.sin(  dt*2*np.pi/(T/2)  )

            # SPIN
            elif self._nextstate == 6:
                T = 8
                dt = (rospy.Time.now()-starttime).to_sec()
                cmd_fullcontrol.enable = 1
                cmd_fullcontrol.x[0] = self._home[0]
                cmd_fullcontrol.y[0] = self._home[1]
                cmd_fullcontrol.z[0] = self._home[2]
                cmd_fullcontrol.yaw[0] = dt*2*np.pi/T

            # MOVE TO A POINT
            elif self._nextstate == 7:
                LOOP_PERIOD = 1/LOOP_RATE
                p = np.asarray(self._state[0:3])
                v = np.asarray(self._stateVel[0:3])
                p_inf = np.array([self._home[0], self._home[1]+0.5, self._home[2]+0.5])
                kap = 10
                kav = 5
                acc_d = -kap*(p - p_inf)-kav*v
                acc_max = 3.0
                acc_d = acc_max*np.tanh(acc_d/acc_max) 
                print(acc_d)

                p_d = p + v*LOOP_PERIOD + 0.5*(LOOP_PERIOD**2)*acc_d
                v_d = v + acc_d*LOOP_PERIOD

                dt = (rospy.Time.now()-starttime).to_sec()
                cmd_fullcontrol.enable = 1

                cmd_fullcontrol.x[0] = p_d[0] 
                cmd_fullcontrol.y[0] = p_d[1]
                cmd_fullcontrol.z[0] = p_d[2]
                
                cmd_fullcontrol.x[1] = v_d[0]
                cmd_fullcontrol.y[1] = v_d[1]
                cmd_fullcontrol.z[1] = v_d[2]

                cmd_fullcontrol.x[2] = acc_d[0] 
                cmd_fullcontrol.y[2] = acc_d[1] 
                cmd_fullcontrol.z[2] = acc_d[2] 

            self._cmdvel_pub.publish(cmd)
            self._cmdfullcontrol_pub.publish(cmd_fullcontrol)
            

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                # EMERGENCY
                if i == 1 and data.buttons[i] == 1:
                    self._nextstate = 1
                    rospy.loginfo("next state is emergency")
                # TAKEOFF TO HOVER
                if i == 0 and data.buttons[i] == 1:
                    self._nextstate = 2
                    rospy.loginfo("next state is hovering")
                # LANDING
                if i == 3 and data.buttons[i] == 1:
                    self._nextstate = 3
                    rospy.loginfo("next state is landing")
                # FIGURE 8 TRAJECTORY
                if i == 2 and data.buttons[i] == 1:
                    self._nextstate = 5
                    rospy.loginfo("next state is tracking 8")
                # SPIN
                if i == 4 and data.buttons[i] == 1:
                    self._nextstate = 6
                    rospy.loginfo("next state is spinning")
                # Move to a point
                if i == 8 and data.buttons[i] == 1:
                    self._nextstate = 7
                    rospy.loginfo("next state is moving to a point")
                # ENABLE POWER
                if i == 7 and data.buttons[i] == 1:
                    rospy.loginfo("enable power")
                    rospy.set_param("enable_power/enable_power", 1)
                    self._update_params(["enable_power/enable_power"])
                # DISABLE POWER
                if i == 6 and data.buttons[i] == 1:
                    rospy.loginfo("Disable power")
                    rospy.set_param("enable_power/enable_power", 0)
                    self._update_params(["enable_power/enable_power"])
                # RESET
                if i == 5 and data.buttons[i] == 1:
                    rospy.loginfo("reset")
                    self._home = self._state[0:3]

                    SE3_xkr = 0.006# 0.00355 
                    SE3_xkw = 0.00002# 0.0000175 
                    SE3_ykr = 0.006
                    SE3_ykw = 0.00002
                    SE3_zkr = 0.0072 # 0.0023
                    SE3_zkw = 0.000028 # 0.00001
                    SE3_kp = 0.25
                    SE3_kd = 0.1

                    rospy.set_param("kalman/resetEstimation", data.buttons[i])
                    self._update_params(["kalman/resetEstimation"])

                    rospy.set_param("SE3_K/SE3_xkr", SE3_xkr) #0.0005
                    self._update_params(["SE3_K/SE3_xkr"])
                    rospy.set_param("SE3_K/SE3_xkw", SE3_xkw) #0.000004
                    self._update_params(["SE3_K/SE3_xkw"])

                    rospy.set_param("SE3_K/SE3_ykr", SE3_ykr) #0.0005
                    self._update_params(["SE3_K/SE3_ykr"])
                    rospy.set_param("SE3_K/SE3_ykw", SE3_ykw) #0.000004
                    self._update_params(["SE3_K/SE3_ykw"])

                    rospy.set_param("SE3_K/SE3_zkr", SE3_zkr)
                    self._update_params(["SE3_K/SE3_zkr"])
                    rospy.set_param("SE3_K/SE3_zkw", SE3_zkw)
                    self._update_params(["SE3_K/SE3_zkw"])

                    rospy.set_param("SE3_K/SE3_kp", SE3_kp) #0.09 0.3
                    self._update_params(["SE3_K/SE3_kp"])
                    rospy.set_param("SE3_K/SE3_kd", SE3_kd) # 0.0003 0.0023
                    self._update_params(["SE3_K/SE3_kd"]) # 0.012

        self._buttons = data.buttons

    def _stateChanged(self, state, enable_logging):
        if enable_logging == 1:
            self._state = state.values
        else:
            self._state = [state.point.x, state.point.y, state.point.z]

    def _stateVelChanged(self, stateVel):
        self._stateVel = stateVel.values

if __name__ == '__main__':
    rospy.init_node('crazyflie_controller', anonymous=True)

    name = rospy.get_namespace().strip('/')
    worldFrame = rospy.get_param("~worldFrame", "world")
    frame = rospy.get_param("~frame")
    joy_topic = rospy.get_param("~joy_topic", "joy")

    controller = Controller(name, joy_topic, frame, worldFrame)
    controller.run()
    rospy.spin()
