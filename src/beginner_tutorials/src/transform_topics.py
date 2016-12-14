#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData, FullControl
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
import numpy 

def transform_control(msg, torque_pub):
    new_msg = GenericLogData()
    new_msg = msg
    new_msg.values[4] = 1000*msg.values[4]
    new_msg.values[5] = 1000*msg.values[5]
    new_msg.values[6] = 1000*msg.values[6]
    torque_pub.publish(new_msg)
    

if __name__ == '__main__':
    rospy.init_node('transform_topics', anonymous=True)
    name = rospy.get_namespace().strip('/')
    torque_pub = rospy.Publisher('/{}/scaledtorque'.format(name), GenericLogData, queue_size=10)
    rospy.Subscriber('/{}/finalControl'.format(name), GenericLogData, transform_control)



