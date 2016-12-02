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
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial.distance import cdist
  
def setGraph( N, edgeList ):
    W = np.zeros((N,N))
    for e in edgeList:
        i = e[0]
        j = e[1]
        W[i][j] = 1
        W[j][i] = 1       
    return W

def setIncidence( N, edgeList ):
    m = len(edgeList)
    B = np.zeros((N,m))
    for idx, e in enumerate(edgeList):
        i = e[0]
        j = e[1]
        B[i,idx] = 1
        B[j,idx] = -1
    return B

class distributed_observer:
    def __init__(self, N, K_kal, Hn, joy_topic, enable_dis_obs):
        self.init = False
        self.N = N
        self.K_p = K_kal[0]
        self.K_v = K_kal[1] 
        self.Hn = Hn
        self.enable_dis_obs = enable_dis_obs
        self.pos = np.zeros((n*N,1))
        self.vel = np.zeros((n*N,1))
        self.time = rospy.get_time()
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        self.subscribers = []
        for i in range (0, N):
            subscriber = Subscriber("/crazyflie"+str(i)+"/trackedPos",PointStamped) 
            self.subscribers.append(subscriber)
        self.trackedPos_sub = ApproximateTimeSynchronizer(self.subscribers,queue_size=2, slop=0.001)
        self.trackedPos_sub.registerCallback(self.estimate)

        self.ext_publishers = []
        for i in range(0, N):
            if enable_dis_obs == 0:
                publisher = rospy.Publisher("/crazyflie"+str(i)+"/estPos",PointStamped,queue_size=10)
            else:
                publisher = rospy.Publisher("/crazyflie"+str(i)+"/external_position",PointStamped,queue_size=10)
            self.ext_publishers.append(publisher) 

    def estimate(self, *messages):
        pos = []
        for msg in messages:
            pos = np.concatenate((pos, (msg.point.x, msg.point.y, msg.point.z)),axis=0)
        z = np.dot(self.Hn, pos) 
        z = np.expand_dims(z, axis=1)
        self.prior_update()
        self.measu_update(z)

        for i in range(0, self.N):
            header = messages[i].header
            point = Point()
            point.x = self.pos[3*i]
            point.y = self.pos[3*i+1]
            point.z = self.pos[3*i+2]

            msg_estPos = PointStamped(header=header, point=point)
            self.ext_publishers[i].publish(msg_estPos)

    def prior_update(self):
        now = rospy.get_time()
        dt = now - self.time
        self.time = now
        self.pos = self.pos + dt*self.vel
        self.vel = self.vel + dt*0
    
    def measu_update(self, z):
        error = z - np.dot(self.Hn, self.pos) 
        self.pos = self.pos + np.dot(self.K_p, error)
        self.vel = self.vel + np.dot(self.K_v, error)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if i == 11 and data.buttons[i] == 1:
                self.init = False
                self.pos = np.zeros((n*N,1))
                self.vel = np.zeros((n*N,1))

if __name__=='__main__':
    rospy.init_node('distributed_observer', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy") 
    enable_dis_obs = rospy.get_param("~enable_dis_obs")
    # dimension
    n = 3
    # num of quads
    N = rospy.get_param("~num_quad") 
    if N == 4:        
        edgeList = [(0,1),
                    (1,2),
                    (2,3)]
    else:
        edgeList = []

    W = setGraph(N, edgeList)
    Dout = np.diag(np.sum(W, axis=1))
    L = Dout - W 
    B = setIncidence(N, edgeList)

    if N == 4:
        idx_ap = [0,3]
    else:
        idx_ap = [0]
    E = np.identity(N)[:,idx_ap]
    H = np.concatenate((B.T, E.T), axis=0)

    In = np.identity(n)
    Hn = np.kron(H, In)     
    # gains
    kalman_p = 0.8
    kalman_v = 2*kalman_p
    kalman_abs = 1.0
    kalman_rel = kalman_abs

    # estimator matrix
    _K_kalp = np.concatenate((kalman_rel*B, kalman_abs*E), axis=1)
    _K_kalp = np.dot(np.diag(1/np.sum(np.abs(_K_kalp),axis=1)),_K_kalp)
    K_kalp = np.kron(_K_kalp, In)
    K_kal = [kalman_p*K_kalp, 
             kalman_v*K_kalp]

    observer = distributed_observer(N, K_kal, Hn, joy_topic, enable_dis_obs)
    rospy.spin()    




 
