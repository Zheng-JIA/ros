#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers
def callback(message):
  rospy.loginfo(rospy.get_caller_id() + 'x position is %f', message.markers[0].translation.x)

def vicon_listener():
  rospy.init_node('vicon_listener', anonymous=True)
  rospy.Subscriber('vicon/markers', Markers, callback)
  rospy.spin()

if __name__=='__main__':
  vicon_listener()

