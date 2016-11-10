#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from vicon_bridge.msg import Markers
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import cPickle
import tf
import numpy as np
from scipy.spatial.distance import cdist
class kalmanFilter:
    def __init__(self, A, k, idx, joy_topic):
        self.init = False 
        self.A = A
        self.state_x = np.array([[0], [0]])
        self.state_y = np.array([[0], [0]])
        self.state_z = np.array([[0], [0]])
        self.k = k
        self.time = rospy.get_time()
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        self.pub_trackedPos = rospy.Publisher('crazyflie'+str(idx)+'/trackedPos',PointStamped,queue_size=10)
        self.pub_extPos = rospy.Publisher('crazyflie'+str(idx)+'/external_position',PointStamped,queue_size=10)
             
    def prior_update(self):
        now = rospy.get_time()
        dt = now - self.time 
        self.time = now
        self.A[0][1] = dt 
        self.state_x = np.dot(self.A, self.state_x)
        self.state_y = np.dot(self.A, self.state_y)
        self.state_z = np.dot(self.A, self.state_z)

    def measu_update(self, i, z1, z2):
        z = self.nn(z1, z2)
        self.state_x = self.state_x + np.dot(self.k, (z[0] - self.state_x[0][0]))
        self.state_y = self.state_y + np.dot(self.k, (z[1] - self.state_y[0][0]))
        self.state_z = self.state_z + np.dot(self.k, (z[2] - self.state_z[0][0]))

    def get_pos(self):
        return (self.state_x[0][0], self.state_y[0][0], self.state_z[0][0])

    def nn(self, z1, z2):
        dist = cdist([self.get_pos()], [z1, z2], 'euclidean')
        idx = np.argmin(dist) 
        z = (z1, z2)  
        return z[idx] 
    
    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if i == 11 and data.buttons[i] == 1:
                self.init = False


def convert_to_rviz_tf(msg, kf):
    br = tf.TransformBroadcaster()
    num_quad = len(kf) 
    num_marker = len(msg.markers)
    pos = []
    """
    if num_marker == 0:
        rospy.loginfo("The number of markers is 0")
    if num_marker == 1:
        rospy.loginfo("The number of markers is 11111111111111111")
    if num_marker == 2:
        rospy.loginfo("The number of markers is 22222222222222222          22222222222222222")
    if num_marker == 3:
        rospy.loginfo("The number of markers is 33333333333333333          33333333333333333           333333333333333")
    """
    # all available measurements
    for i in range(0, num_marker):
        pos.append((msg.markers[i].translation.x/1000.0, msg.markers[i].translation.y/1000.0, msg.markers[i].translation.z/1000.0))
        #br.sendTransform(pos[i], tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(), str(i)+"_marker","world")

    # save valid measurements
    if num_marker == num_quad: 
        pos0 = (msg.markers[0].translation.x/1000.0, msg.markers[0].translation.y/1000.0, msg.markers[0].translation.z/1000.0)
        pos1 = pos0
        if num_quad == 2:
            pos1 = (msg.markers[1].translation.x/1000.0, msg.markers[1].translation.y/1000.0, msg.markers[1].translation.z/1000.0)

    #--------kalman filter estimation--------# 
    if kf[0].init == False and num_marker==num_quad:
        if num_quad == 1:
            kf[0].state_x = np.array([[pos0[0]], [0]])
            kf[0].state_y = np.array([[pos0[1]], [0]])
            kf[0].state_z = np.array([[pos0[2]], [0]])
            kf[0].init = True
             
        if num_quad == 2:
            if pos1[1] > pos0[1]:
                kf[0].state_x = np.array([[pos0[0]], [0]])
                kf[0].state_y = np.array([[pos0[1]], [0]])
                kf[0].state_z = np.array([[pos0[2]], [0]])
                kf[1].state_x = np.array([[pos1[0]], [0]])
                kf[1].state_y = np.array([[pos1[1]], [0]])
                kf[1].state_z = np.array([[pos1[2]], [0]])
            else:
                kf[0].state_x = np.array([[pos1[0]], [0]])
                kf[0].state_y = np.array([[pos1[1]], [0]])
                kf[0].state_z = np.array([[pos1[2]], [0]])
                kf[1].state_x = np.array([[pos0[0]], [0]])
                kf[1].state_y = np.array([[pos0[1]], [0]])
                kf[1].state_z = np.array([[pos0[2]], [0]])
        
            kf[0].init = True
            kf[1].init = True
    for i in range(num_quad):
        kf[i].prior_update()
        if num_marker == num_quad :
            kf[i].measu_update(i, pos0, pos1)
        #br.sendTransform(kf[i].get_pos(), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"crazyflie"+str(i)+"/base_link","world") # unknown reason for affecting system
    #==================================Publish tracked positions=========================================== 
    for i in range(0, num_quad):
        header = msg.header
        point = Point()
        point.x = kf[i].get_pos()[0]
        point.y = kf[i].get_pos()[1]
        point.z = kf[i].get_pos()[2]
        msg_extPos = PointStamped(header=header, point=point)
        kf[i].pub_trackedPos.publish(msg_extPos)
   
    #==================================Publish /crazyflie/external_position================================
    for i in range(0, num_quad):
        header = msg.header
        point = Point()
        point.x = kf[i].get_pos()[0]
        point.y = kf[i].get_pos()[1]
        point.z = kf[i].get_pos()[2]
        msg_extPos = PointStamped(header=header, point=point)
        kf[i].pub_extPos.publish(msg_extPos)
    """
    #scaled0 = Point()
    #scaled1 = Point()
    #scaled0.x = msg.markers[0].translation.x/1000.0
    #scaled0.y = msg.markers[0].translation.y/1000.0
    #scaled0.z = msg.markers[0].translation.z/1000.0
    #msg_scaled0 = PointStamped(header=header, point=scaled0)

    #scaled1.x = msg.markers[1].translation.x/1000.0
    #scaled1.y = msg.markers[1].translation.y/1000.0
    #scaled1.z = msg.markers[1].translation.x/1000.0
    #pub_vicon.publish(msg_scaled0)
    """
if __name__=='__main__':
    rospy.init_node('vicon_rviz_marker', anonymous=True)
    A = np.array([[1, 0.005],[0, 1]])
    k = np.array([[0.3], [0.00006]]) # 0.3 0.00006
    joy_topic = rospy.get_param("~joy_topic", "joy")
    kf0 = kalmanFilter(A, k, 0, joy_topic)
    kf1 = kalmanFilter(A, k, 1, joy_topic)
    kf = [kf0]
    pub_vicon = rospy.Publisher('scaledVicon', PointStamped, queue_size=10)
    rospy.Subscriber('/vicon/markers',
                    Markers,
                    convert_to_rviz_tf,
                    kf)
    rospy.spin()
