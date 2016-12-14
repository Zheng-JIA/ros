#!/usr/bin/env python
import roslib
#roslib.load_manifest('crazyflie_ros_apps')

import rospy
import tf
from crazyflie_driver.msg import GenericLogData

from struct import unpack, pack
from math import radians

NAME = None
FRAME = None
WORLDFRAME = None
def publish_crazyflie_state(state):
    br = tf.TransformBroadcaster()
    global x,y,z
    x,y,z,roll,pitch,yaw = state
    br.sendTransform(
        (x,y,z),
        #tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw)),
        tf.transformations.quaternion_from_euler(roll, pitch, yaw),
        rospy.Time.now(),
        '{}/{}'.format(NAME,FRAME),
        WORLDFRAME)

def handle_crazyflie_state(state):
    publish_crazyflie_state(state.values)

def send_desired_roll_pitch_yaw(setpoint):
    br = tf.TransformBroadcaster()
    global x,y,z
    br.sendTransform(
            (x,y,z),
            tf.transformations.quaternion_from_euler(setpoint.values[0],setpoint.values[1],setpoint.values[2]),
            rospy.Time.now(),
            '{}/{}'.format(NAME,"Att_Desired"),
            WORLDFRAME)


if __name__ == '__main__':
    rospy.init_node('crazyflie_state_broadcaster')

    NAME = rospy.get_namespace().strip('/')
    FRAME = rospy.get_param('~frame', 'frame')
    WORLDFRAME = rospy.get_param('~worldFrame', 'world')
    global x,y,z
    x = 0
    y = 0
    z = 0
    rospy.Subscriber(
        '/'+NAME+'/state',
        GenericLogData,
        handle_crazyflie_state)

    rospy.Subscriber(
        '/'+NAME+'/setpoint',
        GenericLogData,
        send_desired_roll_pitch_yaw)
    rospy.spin()
