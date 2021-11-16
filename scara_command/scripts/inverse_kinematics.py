#!/usr/bin/env python
import rospy
import numpy as np
from scara_command.srv import InverseKinematics, InverseKinematicsResponse


def handleIKService(msg):

    d1 = 1
    a1 = 1
    a2 = 1
    d3 = 1

    x = msg.x
    y = msg.y
    z = msg.z

    alpha  = np.arctan2(-x, y)
    dBeta  = (a1**2 + (x**2 + y**2) - a2**2)/(2*a1*np.sqrt(x**2 + y**2))
    beta   = np.arctan2(np.sqrt(1 - dBeta**2), dBeta) # +/-
    t1     = alpha + beta # +/-

    dGamma = (a2**2 + a1**2 - (x**2 + y**2))/(2*a2*a1)
    t2     = np.arctan2(np.sqrt(1 - dGamma**2), dGamma) - np.pi # +/-

    t3     = d1 - d3 + z

    q = (np.rad2deg(t1), np.rad2deg(t2), t3)

    return InverseKinematicsResponse(q)


def main():

    rospy.init_node('ik_respond', anonymous=True)
    service = rospy.Service('inverse_kinematics', InverseKinematics, handleIKService)

    rospy.spin()


if __name__=="__main__":
    main()
