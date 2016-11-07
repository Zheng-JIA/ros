#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from vicon_bridge.msg import Markers
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import tf
import numpy as np
from scipy.spatial.distance import cdist
class kalmanFilter:
    def __init__(self, A, k):
        self.init = False 
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

    def measu_update(self, z1, z2):
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

def convert_to_rviz_tf(msg, kf):
    br = tf.TransformBroadcaster()
    num_quad = len(kf) 
    num_marker = len(msg.markers)
    pos = []
    for i in range(0, num_marker):
        pos.append((msg.markers[i].translation.x/1000.0,
                msg.markers[i].translation.y/1000.0,
                msg.markers[i].translation.z/1000.0))
        if num_marker == 1:
            rospy.loginfo("The number of markers is %f", num_marker)
        if num_marker == 3:
            rospy.loginfo("There are three markers")
        #br.sendTransform(pos[i], tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(), str(i)+"_marker","world")
        #br.sendTransform(pos[i], tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(), "crazyflie"+str(i)+"/base_link","world")

    pos1 = (msg.markers[0].translation.x/1000.0,
                msg.markers[0].translation.y/1000.0,
                msg.markers[0].translation.z/1000.0)
    if num_marker == 2: 
        pos2 = (msg.markers[1].translation.x/1000.0,
                msg.markers[1].translation.y/1000.0,
                msg.markers[1].translation.z/1000.0)
    else:
        pos2 = pos1
    #"""
    #--------kalman filter estimation--------# 
    if kf[0].init == False:
        kf[0].state_x = np.array([[pos1[0]], [0]])
        kf[0].state_y = np.array([[pos1[1]], [0]])
        kf[0].state_z = np.array([[pos1[2]], [0]])
        kf[1].state_x = np.array([[pos2[0]], [0]])
        kf[1].state_y = np.array([[pos2[1]], [0]])
        kf[1].state_z = np.array([[pos2[2]], [0]])
        kf[0].init = True
        kf[1].init = True
    kf[0].prior_update()
    kf[1].prior_update()
    if num_marker == 2 :
        kf[0].measu_update(pos1, pos2)
        kf[1].measu_update(pos1, pos2)
    #br.sendTransform(kf[0].get_pos(), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"crazyflie0/base_link","world")
    #br.sendTransform(kf[1].get_pos(), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"crazyflie1/base_link","world")

    header = msg.header
    point0 = Point()
    point1 = Point()
    point0.x = kf[0].get_pos()[0]
    point0.y = kf[0].get_pos()[1]
    point0.z = kf[0].get_pos()[2]
    point1.x = kf[1].get_pos()[0]
    point1.y = kf[1].get_pos()[1]
    point1.z = kf[1].get_pos()[2]

    msg_extPos0 = PointStamped(header=header, point=point0)
    msg_extPos1 = PointStamped(header=header, point=point1)
    pub_extPos0.publish(msg_extPos0)
    pub_extPos1.publish(msg_extPos1)
if __name__=='__main__':
    rospy.init_node('vicon_rviz_marker', anonymous=True)
    A = np.array([[1, 0.005],[0, 1]])
    k = np.array([[0.3], [0.00006]]) # 0.3 0.00006
    kf0 = kalmanFilter(A, k)
    kf1 = kalmanFilter(A, k)
    pub_extPos0 = rospy.Publisher('crazyflie0/external_position',PointStamped,queue_size=10)
    pub_extPos1 = rospy.Publisher('crazyflie1/external_position',PointStamped,queue_size=10)
    rospy.Subscriber('/vicon/markers',
                    Markers,
                    convert_to_rviz_tf,
                    (kf0, kf1))
    rospy.spin()
