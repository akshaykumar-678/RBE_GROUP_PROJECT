

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray 
from sensor_msgs.msg import JointState
import numpy as np
import math as m
from math import sin, cos, sqrt
global pub
pub=rospy.Publisher("FK",Float64MultiArray,queue_size=10)

def callback(position):
    q1 = position.position[0]
    q2 = position.position[1]
    q3 = position.position[2]

    return q1,q2,q3

    #rospy.loginfo("X=%d\n Y=%d\n Z=%d\n", q1, q2, q3)


def forw_kine(position):
	q1 = position.position[0]
	q2 = position.position[1]
	q3 = position.position[2]

	t1 = np.matrix([[cos(q1), 0, sin(q1), 0], [sin(q1), 0, -cos(q1), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
	t2 = np.matrix([[cos(q2), sin(q2), 0, sin(q2)], [sin(q2), cos(q2), 0, cos(q2)], [0, 0, 1, 0], [0, 0, 0, 1]])
	t3 = np.matrix([[cos(q3), -sin(q3), 0, cos(q3)], [sin(q3),cos(q3), 0, sin(q3)], [0, 0, 1, 0], [0, 0, 0, 1]])
	kine = np.matmul(t1, t2, t3)
	pub.publish(kine)
	#print("Kinematics are ",kine,"\n")
	return kine

def listener():
    rospy.init_node('FK', anonymous=True)
    sub=rospy.Subscriber("/scara/joint_states", JointState, forw_kine)
    #pub=rospy.Publisher("FK",Float64MultiArray,queue_size=10)
    rate=rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__':
    listener()
