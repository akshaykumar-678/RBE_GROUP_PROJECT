#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray 

def dhparam(theta, d, a, alpha):

    theta = m.radians(theta)
    r_z = np.array([[round(m.cos(theta),3), round(-m.sin(theta),3), 0, 0], [round(m.sin(theta),3), round(m.cos(theta),3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    t_z = np.array([[1, 0, 0, 0], [0, 1, 0,0], [0, 0, 1, d], [0, 0, 0, 1]])
    t_x = np.array([[1, 0, 0, a], [0, 1, 0,0], [0, 0, 1, 0], [0, 0, 0, 1]])
    r_x = np.array([[1, 0, 0, 0], [0, round(m.cos(alpha),3), round(-m.sin(alpha),3), 0 ], [0, round(m.sin(alpha),3), round(m.cos(alpha),3), 0 ], [0, 0, 0, 1]])

    t = np.matmul(t_x, r_x)
    t = np.matmul(t_z, t)
    t = np.matmul(r_z, t)

    return t

def forward_kin(joint_values):

    a_1 = 2
    a_2 = 2
    a_3 = 0.4
    a_4 = 2
    a_5 = 1
    
    theta = np.array([joint_values[1], joint_values[2], 0])
    alpha = np.array([0, m.pi, 0])
    d = np.array([a_1, a_3, a_5 + joint_values[0]])
    a = np.array([a_2, a_4, 0])

    t = []
    for i in range(0 , 3):
        t.append(dhparam(theta[i], d[i], a[i], alpha[i]))

    
    k = t[0]
    for i in range(1 , 3):
        k = np.matmul(k , t[i])

    return k

def callback(data):

    # print ("the positions are: ", data.position,"\n")
    joint_values = data.position
    pose = forward_kin(joint_values)
    # print(pose)

    # Defining the publisher
    pub = rospy.Publisher('fwd_kin', Float64MultiArray, queue_size=10)
    
    x = pose[0,3]
    y = pose[1,3]
    z = pose[2,3]

    roll = m.degrees(m.atan2(pose[2, 1], pose[2, 2]))
    pitch = m.degrees(m.atan2((-1 * pose[2, 0]), m.sqrt(pow(pose[2, 1], 2) + pow(pose[2, 2], 2))))
    yaw = m.degrees(m.atan2(pose[1, 0], pose[0, 0]))

    forward = np.array([x , y, z, roll, pitch, yaw])
    
#   Creating an object for the message type Float64MultiArray	
    fwd = Float64MultiArray()
    fwd.data = forward
    pub.publish(fwd)

def listner():

    rospy.init_node('listner', anonymous = True)
    
    # Defining the subscriber
    rospy.Subscriber("/scara/joint_states",JointState, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    listner()
