

#!/usr/bin/env python
from os import F_OK
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray 

from sensor_msgs.msg import JointState
import numpy as np
import math as m
from math import sin, cos, sqrt,atan2
FK=[]
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

	t1 = np.matrix([[cos(q1+q2), -sin(q1+q2), 0,2*cos(q1)+2*cos(q1+q2)], [sin(q1+q2), cos(q1+q2), 0, 2*sin(q1)+2*sin(q1+q2)], [0, 0, -1, 2-q3], [0, 0, 0, 1]])
	x=t1[0,3]
	y=t1[1,3]
	z=t1[2,3]
	roll = atan2(t1[2, 1], t1[2, 2])
	pitch = atan2((-1 * t1[2, 0]), sqrt(pow(t1[2, 1], 2) + pow(t1[2, 2], 2)))
	yaw = atan2(t1[1, 0], t1[0, 0])
	FK=[x,y,z,roll,pitch,yaw]
	#print (type(FK))
	return FK
def listener():
    rospy.init_node('FK', anonymous=True)
    pub=rospy.Publisher("FK",Float32MultiArray,queue_size=10)
    sub=rospy.Subscriber("/scara/joint_states", JointState, forw_kine)
    print(FK)
    #pub.publish(FK)
    rate=rospy.Rate(10)
    rospy.spin()
    


if __name__ == '__main__':
    listener()
