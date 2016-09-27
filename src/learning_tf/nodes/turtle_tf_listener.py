#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import time
if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    # wait for service advised by the server
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    # wait for the spawn of carrot1
    start = rospy.Time.now()
    listener.waitForTransform("/turtle2","/turtle1",rospy.Time(), rospy.Duration(4.0))  
    # since we need transformation from 3 seconds ago, we need to wait for 3 secs otherwise there will be tf.Exception thrown.
    rospy.sleep(3.0)
    end = rospy.Time.now()
    waitTime = end - start
    rospy.loginfo("duration is %f"%waitTime.to_sec())
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            past = now - rospy.Duration(3.0)
            listener.waitForTransformFull("/turtle2", now, "/turtle1", past, "/world", rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransformFull("/turtle2", now,"/turtle1", past, "/world")
        
        except ( tf.LookupException, tf.ConnectivityException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
