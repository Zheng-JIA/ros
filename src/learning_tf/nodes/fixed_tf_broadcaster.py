#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
import math
if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((2.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0,0,math.pi/2.0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")
        rate.sleep()
