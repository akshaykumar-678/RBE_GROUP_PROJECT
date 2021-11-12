#!/usr/bin/env python3
import rospy
import numpy as np
import math as m
from math import sin,cos,sqrt
import geometry_msgs.msg as GM
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
l1=1
l2=1
l3=1
def forw_kine(dat):
	q1=dat.linear.x
	q2=dat.linear.y
	q3=dat.linear.z

	t1=np.matrix([[cos(q1),0,sin(q1),0],
				[sin(q1), 0, -cos(q1),0],
				[0,1, 0,0],
				[0,0,0,1]])
	t2=np.matrix([[cos(q2),sin(q2),0,sin(q2)],
				[sin(q2), cos(q2),0,cos(q2)],
				[0,0, 1,0],
				[0,0,0,1]])
	t3=np.matrix([[cos(q3),-sin(q3),0,cos(q3) ],
				[sin(q3), cos(q3),0,sin(q3)],	
				[0,0, 1,0],
				[0,0,0,1]])
	kine=t1*(t2*t3)	

	print("\n",kine,"\n")
if __name__=='__main__':
	try:
		rospy.init_node('Sub')
		rospy.Subscriber('forw_kine',GM.Twist,forw_kine)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
