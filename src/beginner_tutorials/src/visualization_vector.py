#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist, Point
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData, FullControl
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import numpy 

def pub_marker(msg):
    global pos
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.ARROW
    (start, end) = (Point(), Point())
    start.x = pos[0]
    start.y = pos[1]
    start.z = pos[2]
    end.x = start.x + 10*msg.values[0]
    end.y = start.y + 10*msg.values[1]
    end.z = start.z + 10*msg.values[2]
    marker.points.append(start)
    marker.points.append(end)
    marker.scale.x = 0.05
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    pub.publish(marker) 

def save_position(msg):
    global pos 
    pos[0] = msg.values[0]
    pos[1] = msg.values[1]
    pos[2] = msg.values[2]

if __name__ == '__main__':
    pos = [None, None, None]
    rospy.init_node("visualization_vector")
    rospy.Subscriber("crazyflie0/Fd", GenericLogData, pub_marker)
    rospy.Subscriber("crazyflie0/state", GenericLogData, save_position)
    pub = rospy.Publisher("visualize_Fd", Marker, queue_size=10)
    rospy.spin()
