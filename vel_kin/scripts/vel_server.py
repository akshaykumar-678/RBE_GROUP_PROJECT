#!/usr/bin/env python

from vel_kin.srv import VelKinJointEndeff,VelKinJointEndeffResponse, VelKinEndeffJoint, VelKinEndeffJointResponse
import rospy
import numpy as np 
import math as m 

class V_kin(object):

    def __init__(self, joint_values):
        
        self.joint_values = joint_values
        # print(self.joint_values)
        # initializing dh-parameter table
        self.a_1 = 2.0
        self.a_2 = 2.0
        self.a_3 = 0.4
        self.a_4 = 2.0
        self.a_5 = 1.0

        self.theta = np.array([joint_values[0], joint_values[1],0]) 
        self.alpha = np.array([0, m.pi, 0])
        self.d = np.array([self.a_1, self.a_3, self.a_5 + joint_values[2]])
        self.a = np.array([self.a_2, self.a_4, 0])
        
    def fwd_kin(self):

        theta = self.theta
        alpha = self.alpha
        d = self.d
        a = self.a
        

        # Calculated Ai transformation matrix gets appended as list of matrix
        t = []
        for i in range(0 , 3):
            t.append(self.dhparam(theta[i], d[i], a[i], alpha[i]))
            
        return t

    def dhparam(self, theta, d, a, alpha):


        # theta = m.radians(theta)
        r_z = np.array([[round(m.cos(theta),3), round(-m.sin(theta),3), 0, 0], [round(m.sin(theta),3), round(m.cos(theta),3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        t_z = np.array([[1, 0, 0, 0], [0, 1, 0,0], [0, 0, 1, d], [0, 0, 0, 1]])
        t_x = np.array([[1, 0, 0, a], [0, 1, 0,0], [0, 0, 1, 0], [0, 0, 0, 1]])
        r_x = np.array([[1, 0, 0, 0], [0, round(m.cos(alpha),3), round(-m.sin(alpha),3), 0 ], [0, round(m.sin(alpha),3), round(m.cos(alpha),3), 0 ], [0, 0, 0, 1]])

        t = np.matmul(t_x, r_x)
        t = np.matmul(t_z, t)
        t = np.matmul(r_z, t)

        # print(t)
        
        return t

    def jacobian(self, transformation_list, joint_type, joint_no): 

        t_1_0 = transformation_list[0]
        a_2_1 = transformation_list[1]
        a_3_2 = transformation_list[2]
        # print(t_1_0)

        t_2_0 = np.matmul(t_1_0, a_2_1)
        t_3_0 = np.matmul(t_2_0, a_3_2)
        # print("ji")
        # print(t_1_0)
        # print(t_2_0)
        # print(t_3_0)

        homogenous_transformation_list = [t_1_0, t_2_0, t_3_0]
        # print(homogenous_transformation_list)
        

        if joint_no == 1:
            if joint_type == "revolute":
                
                return self.jacobi_revolute(homogenous_transformation_list, 1)
            
            elif joint_type == "prismatic":
                
                return self.jacobi_prismatic(homogenous_transformation_list, 1)
            
            else:
                print("Entered wrong type")

        
        elif joint_no == 2:
            if joint_type == "revolute":
               
                return self.jacobi_revolute(homogenous_transformation_list, 2)
            
            elif joint_type == "prismatic":
                
                return self.jacobi_prismatic(homogenous_transformation_list, 2)
            
            else:
                print("Entered wrong type")

                
        elif joint_no == 3:
            if joint_type == "revolute":
                
                return self.jacobi_revolute(homogenous_transformation_list, 3)
            
            elif joint_type == "prismatic":
                
                return self.jacobi_prismatic(homogenous_transformation_list, 3)
            
            else:
                print("Entered wrong type")

        else:
            print("Entered wrong joint_no")

    def jacobi_revolute(self, homo_trans_list, joint_no):
        
        t_3_0 = homo_trans_list[2]
        t_i_0 = homo_trans_list[joint_no-1]
        o_n = t_3_0[:3,3]
        
        if(joint_no == 1):

            o_i = np.array([0, 0, 0])
        else:
            o_i = t_i_0[:3,3]
        
        
        o_d = o_n - o_i

        z_i = t_i_0[:3,2]
        
        # angular component
        z_i_t = np.transpose(np.array([z_i]))

        # linear component
        cross = np.cross(z_i, o_d)
        cross_t = np.transpose(np.array([cross]))

        return np.vstack((cross_t, z_i_t))

    def jacobi_prismatic(self, homo_trans_list, joint_no):

        t_i_0 = homo_trans_list[joint_no-1]
        z_i = t_i_0[:3,2]

        # linear component
        z_i_t = np.transpose(np.array([z_i]))

        # angular component
        a = np.transpose([np.array([0, 0, 0])])
        return np.vstack((z_i_t, a))

def jacobian_matrix():

    joint_type = ["revolute", "revolute", "prismatic"]
    joint_values = np.array([1.0, 0.5, 2.0])

    # write a subcriber to get joint position values 
    fwd = V_kin(joint_values)
    transformation_list = fwd.fwd_kin()

    for i in range(len(joint_type)):

        temp = fwd.jacobian(transformation_list, joint_type[i], i + 1)
        # print(temp)
        if i == 0:
            
            b = temp

        else:

            b = np.hstack((b, temp))

    return b

def handle_VelKinJointEndeff(req):

    joint_velocities = np.array([[req.q1_dot], [req.q2_dot], [req.q3_dot]])

    jacob_mat = jacobian_matrix()
    endeff_vel = np.matmul(jacob_mat, joint_velocities)
    
    # print(endeff_vel)
    v_x = endeff_vel[0]
    v_y = endeff_vel[1]
    v_z = endeff_vel[2]
    w_x = endeff_vel[3]
    w_y = endeff_vel[4]
    w_z = endeff_vel[5]
    
    return  VelKinJointEndeffResponse(v_x, v_y, v_z, w_x, w_y, w_z)

def handle_VelKinEndeffjoint(req):

    endeff_vel = np.array([[req.v_x], [req.v_y],[req.v_z], [req.w_x], [req.w_y], [req.w_z]])
    
    jacob_mat = jacobian_matrix()    
    jacob_mat_inv = np.linalg.pinv(jacob_mat)
    joint_vel = np.matmul(jacob_mat_inv, endeff_vel)

    q1_dot = joint_vel[0]
    q2_dot = joint_vel[1]
    q3_dot = joint_vel[2]

    return VelKinEndeffJointResponse(q1_dot, q2_dot, q3_dot)

def Conversion_server():
    #init the node
    rospy.init_node('conversion_server_node')

    #init the two service calls
    serv_1 = rospy.Service('scara/convert_jointvel_to_endeffvel', VelKinJointEndeff, handle_VelKinJointEndeff) 
    serv_2 = rospy.Service('scara/convert_endeffvel_to_jointvel', VelKinEndeffJoint, handle_VelKinEndeffjoint)
    print("ready for conversion: ")

    rospy.spin()

if __name__ == "__main__":
    Conversion_server()