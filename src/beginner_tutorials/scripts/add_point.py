#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from vicon_bridge.msg import Markers
import tf
import numpy as np
class kalmanFilter:
    def __init__(self, A, k):
        self.A = A
        self.state_x = np.array([[0], [0]])
        self.state_y = np.array([[0], [0]])
        self.state_z = np.array([[0], [0]])
        self.k = k
        self.time = rospy.get_time()

    def prior_update(self):
        dt = rospy.get_time() - self.time 
        self.A[0][1] = dt 
        self.state_x = np.dot(self.A, self.state_x)
        self.state_y = np.dot(self.A, self.state_y)
        self.state_z = np.dot(self.A, self.state_z)

    def measu_update(self, z):
        self.state_x = self.state_x + np.dot(self.k, (z[0] - self.state_x[0][0]))
        self.state_y = self.state_y + np.dot(self.k, (z[1] - self.state_y[0][0]))
        self.state_z = self.state_z + np.dot(self.k, (z[2] - self.state_z[0][0]))

    def get_pos(self):
        return (self.state_x[0][0], self.state_y[0][0], self.state_z[0][0])

def convert_to_rviz_tf(msg, kf):
    br = tf.TransformBroadcaster()
    pos = (msg.markers[0].translation.x/1000.0,
                msg.markers[0].translation.y/1000.0,
                msg.markers[0].translation.z/1000.0)
    br.sendTransform(pos,tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"first_marker","world")
   
    #--------kalman filter estimation--------# 
    kf.prior_update()
    kf.measu_update(pos)
    br.sendTransform(kf.get_pos(), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"kf_estimation","world")

    #    position2 = (msg.markers[1].translation.x/1000.0,
    #             msg.markers[1].translation.y/1000.0,
    #            msg.markers[1].translation.z/1000.0)
    #br.sendTransform(position2,tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"second_marker","world")
     
    #rospy.loginfo(rospy.get_caller_id() + 'x position is %f',msg.markers[0].translation.x)
if __name__=='__main__':
    rospy.init_node('vicon_rviz_marker', anonymous=True)
    A = np.array([[1, 0.005],[0, 1]])
    k = np.array([[0.3], [0.00006]])
    kf = kalmanFilter(A, k)
    rospy.Subscriber('vicon/markers',
                    Markers,
                    convert_to_rviz_tf,
                    kf)
    rospy.spin()
