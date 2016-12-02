#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from vicon_bridge.msg import Markers
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
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
        self.pub_trackedPos = rospy.Publisher('crazyflie'+str(idx)+'/trackedPos',PointStamped,queue_size=20)
        global enable_dis_obs
        if enable_dis_obs == 0:
            self.pub_extPos = rospy.Publisher('crazyflie'+str(idx)+'/external_position',PointStamped,queue_size=10)
             
    def prior_update(self):
        now = rospy.get_time()
        dt = now - self.time 
        self.time = now
        self.A[0][1] = dt 
        self.state_x = np.dot(self.A, self.state_x)
        self.state_y = np.dot(self.A, self.state_y)
        self.state_z = np.dot(self.A, self.state_z)

    def measu_update(self, pos):
        z = self.nn(pos)
        self.state_x = self.state_x + np.dot(self.k, (z[0] - self.state_x[0][0]))
        self.state_y = self.state_y + np.dot(self.k, (z[1] - self.state_y[0][0]))
        self.state_z = self.state_z + np.dot(self.k, (z[2] - self.state_z[0][0]))

    def get_pos(self):
        return (self.state_x[0][0], self.state_y[0][0], self.state_z[0][0])

    def nn(self, pos):
        dist = cdist([self.get_pos()], pos, 'euclidean')
        idx = np.argmin(dist) 
        return pos[idx] 
    
    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if i == 11 and data.buttons[i] == 1:
                self.init = False


def convert_to_rviz_tf(msg, kf):
    br = tf.TransformBroadcaster()
    num_quad = len(kf) 
    num_marker = len(msg.markers)
    pos = []
    pos_all = []
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
    if num_marker == 3:
        header = msg.header 
        point = Point()
        point.x = 0
        state_led = PointStamped(header=header, point=point)
        pub_state_led.publish(state_led)
    if num_marker == 4:
        header = msg.header
        point = Point()
        point.x = 1
        state_led = PointStamped(header=header, point=point)
        pub_state_led.publish(state_led)
    # all available measurements
    for i in range(0, num_marker):
        pos_all.append((msg.markers[i].translation.x/1000.0, msg.markers[i].translation.y/1000.0, msg.markers[i].translation.z/1000.0))
        #br.sendTransform(pos_all[i], tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(), str(i)+"_marker","world")

    # save valid measurements
    if num_marker == num_quad: 
        for i in range(0, num_marker):
            pos.append((msg.markers[i].translation.x/1000.0, msg.markers[i].translation.y/1000.0, msg.markers[i].translation.z/1000.0))
        pos = np.asarray(pos)
    # kalman filter
    for i in range(num_quad):
        # initialization 
        if kf[i].init == False and num_marker == num_quad:
            pos = pos[np.argsort(pos[:,1])]
            kf[i].state_x = np.array([[ pos[i][0] ], [0]])
            kf[i].state_y = np.array([[ pos[i][1] ], [0]])
            kf[i].state_z = np.array([[ pos[i][2] ], [0]])
            kf[i].init = True
        # prior update
        kf[i].prior_update()
        # measurement update
        if num_marker == num_quad:
            kf[i].measu_update(pos)
        br.sendTransform(kf[i].get_pos(), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"crazyflie"+str(i)+"/base_link","world") # unknown reason for affecting system
    #==================================Publish tracked positions=========================================== 
    global enable_dis_obs
    for i in range(0, num_quad):
        header = msg.header
        point = Point()
        point.x = kf[i].get_pos()[0]
        point.y = kf[i].get_pos()[1]
        point.z = kf[i].get_pos()[2]
        msg_extPos = PointStamped(header=header, point=point)
        kf[i].pub_trackedPos.publish(msg_extPos)

    #==================================Publish /crazyflie/external_position================================
    if enable_dis_obs == 0:
        for i in range(0, num_quad):
            header = msg.header
            point = Point()
            point.x = kf[i].get_pos()[0]
            point.y = kf[i].get_pos()[1]
            point.z = kf[i].get_pos()[2]
            msg_extPos = PointStamped(header=header, point=point)
            kf[i].pub_extPos.publish(msg_extPos)
    scaled0 = Point()
    #scaled1 = Point()
    scaled0.x = msg.markers[0].translation.x/1000.0
    scaled0.y = msg.markers[0].translation.y/1000.0
    scaled0.z = msg.markers[0].translation.z/1000.0
    msg_scaled0 = PointStamped(header=header, point=scaled0)

    #scaled1.x = msg.markers[1].translation.x/1000.0
    #scaled1.y = msg.markers[1].translation.y/1000.0
    #scaled1.z = msg.markers[1].translation.x/1000.0
    pub_vicon.publish(msg_scaled0)
if __name__=='__main__':
    rospy.init_node('vicon_rviz_marker', anonymous=True)
    A = np.array([[1, 0.005],[0, 1]])
    k = np.array([[0.3], [0.00006]]) # 0.3 0.00006
    joy_topic = rospy.get_param("~joy_topic", "joy")
    enable_dis_obs = rospy.get_param("~enable_dis_obs")
    num_quad = rospy.get_param("~num_quad")
    kf = []
    for i in range(0, num_quad):
        kf.append(kalmanFilter(A, k, i, joy_topic))
    
    pub_vicon = rospy.Publisher('scaledVicon', PointStamped, queue_size=10)
    pub_state_led = rospy.Publisher('state_led', PointStamped, queue_size=10)
    rospy.Subscriber('/vicon/markers',
                    Markers,
                    convert_to_rviz_tf,
                    kf)
    rospy.spin()
