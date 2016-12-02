#!/usr/bin/env python
import sys
import rospy
import atexit
import cPickle
from vicon_bridge.msg import Markers
from crazyflie_driver.srv import *
import numpy as np

def exit_handler():
    global delay_all
    f = file("/home/zheng/ros/src/beginner_tutorials/scripts/delay",'wb')
    cPickle.dump(delay_all, f, protocol=cPickle.HIGHEST_PROTOCOL)
    f.close()
    print("Process terminated and filed dumped")
atexit.register(exit_handler)

def get_delay(msg):
    global Sub_led_mode 
    global led_mode 
    global start
    global delay_all
    num_marker = len(msg.markers)
    rospy.loginfo("Sub_led_mode %d, number of marker %d, led_mode %d and start %f", Sub_led_mode, num_marker, led_mode, start)
    if (Sub_led_mode is not num_marker):
        Sub_led_mode = led_mode
        end = msg.header.stamp.to_sec()
        delay = end - start
        delay_all.append(delay)
        rospy.loginfo("the start is %f", start)
        rospy.loginfo("the end is %f", end)
        rospy.loginfo("the delay is %f", delay)

if __name__=='__main__':
    rospy.init_node('toggleLed')
    r = rospy.Rate(10)
    led_mode = 0
    Sub_led_mode = 0
    start = rospy.get_time()
    delay_all = []
    rospy.Subscriber('/vicon/markers',
                         Markers,
                         get_delay
                         )
    while not rospy.is_shutdown():
        led_mode = 1 - led_mode  
        start = rospy.get_time()
        rospy.wait_for_service('update_params')
        _update_params = rospy.ServiceProxy('update_params', UpdateParams)
        rospy.set_param("viconled/led_mode", led_mode)
        # change the parameter led_mode
        _update_params(["viconled/led_mode"])

        #rospy.loginfo("Sub_led_mode is %d", Sub_led_mode)
        #rospy.loginfo("the led_mode is %d", led_mode)
        r.sleep()
