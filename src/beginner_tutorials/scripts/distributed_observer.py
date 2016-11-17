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
    def __init__(self, K_kal, Hn, joy_topic, enable_dis_obs):
        self.init = False
        self.K_p = K_kal[0]
        self.K_v = K_kal[1] 
        self.Hn = Hn
        self.pos = np.zeros((n*N,1))
        self.vel = np.zeros((n*N,1))
        self.time = rospy.get_time()
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        self.trackedPos_sub = ApproximateTimeSynchronizer([Subscriber("/crazyflie0/trackedPos",PointStamped),Subscriber("/crazyflie1/trackedPos",PointStamped)],queue_size=2, slop=0.001)
        self.trackedPos_sub.registerCallback(self.estimate)
        if enable_dis_obs == 0:
            self.estPos_pub0 = rospy.Publisher("/crazyflie0/estPos",PointStamped,queue_size=10)
            self.estPos_pub1 = rospy.Publisher("/crazyflie1/estPos",PointStamped,queue_size=10)
        else:
            self.estPos_pub0 = rospy.Publisher("/crazyflie0/external_position",PointStamped,queue_size=10)
            self.estPos_pub1 = rospy.Publisher("/crazyflie1/external_position",PointStamped,queue_size=10)

    def estimate(self, msg0, msg1):
        pos0 = (msg0.point.x, msg0.point.y, msg0.point.z)
        pos1 = (msg1.point.x, msg1.point.y, msg1.point.z)
        pos = np.concatenate((pos0,pos1),axis=0)
        z = np.dot(self.Hn, pos) 
        self.prior_update()
        self.measu_update(z)
        header = msg0.header
        point = Point()
        point.x = pos0[0]
        point.y = pos0[1]
        point.z = pos0[2]
        msg_estPos = PointStamped(header=header, point=point)
        self.estPos_pub0.publish(msg_estPos)
        header = msg1.header
        point.x = pos1[0]
        point.y = pos1[1]
        point.z = pos1[2]
        msg_estPos = PointStamped(header=header, point=point)
        self.estPos_pub1.publish(msg_estPos)

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



if __name__=='__main__':
    rospy.init_node('distributed_observer', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy") 
    enable_dis_obs = rospy.get_param("~enable_dis_obs")
    n = 3
    N = 2 
        
    edgeList = [(0,1)]
    W = setGraph(N, edgeList)
    Dout = np.diag(np.sum(W, axis=1))
    L = Dout - W 
    B = setIncidence(N, edgeList)

    idx_ap = [0]
    E = np.identity(N)[:,idx_ap]
    H = np.concatenate((B.T, E.T), axis=0)
    In = np.identity(n)
    Hn = np.kron(H, In)     
    # gains
    kalman_p = 0.17*2
    kalman_v = 1*kalman_p
    kalman_abs = 1.0
    kalman_rel = 0.8*kalman_abs

    # estimator matrix
    K_kalp = np.concatenate((kalman_rel*B, kalman_abs*E), axis=1)
    K_kalp = np.kron(K_kalp, In)
    K_kal = [kalman_p*K_kalp, 
             kalman_v*K_kalp]

    observer = distributed_observer(K_kal, Hn, joy_topic, enable_dis_obs)
    rospy.spin()    




 
