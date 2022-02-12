#!/usr/bin/env python
from telnetlib import TM
import rospy
from std_msgs.msg import Float64
import math
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

# 声明变量以供数学表达式
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')                                 # joint angles theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')                                 # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')                                 # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # joint twist angles

# get the goal position from 
x= float(input("input x:"))
y= float(input("input y:"))
z= float(input("input z:"))

print("theta1:",theta1)

# DH Table
dh = {alpha0:      0, a0:      0, d1:  0.75, theta1:        theta1,
      alpha1: -pi/2., a1:   0.35, d2:     0, theta2: -pi/2.+theta2,
      alpha2:      0, a2:   1.25, d3:     0, theta3:        theta3,
      alpha3: -pi/2., a3: -0.054, d4:   1.5, theta4:        theta4,
      alpha4:  pi/2., a4:      0, d5:     0, theta5:        theta5,
      alpha5: -pi/2., a5:      0, d6:     0, theta6:        theta6,
      alpha6:      0, a6:      0, d7: 0.303, theta7:         0}

def Homogeneous_transform_matrix(alpha,a,d,theta):
    HTM = Matrix([[           cos(theta),           -sin(theta),           0,            a],
                 [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                 [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [                    0,                     0,          0,            1]])
    return HTM

# def Rotation_matrix(alpha,theta):
#     RM = Matrix([[           cos(theta),           -sin(theta),           0],
#                  [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha)],
#                  [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha)],
#                  [                    0,                     0,          1]])
#     return RM

def Translation_matrix(alpha,a,d):
    TM = Matrix([[a],
                [-sin(alpha)*d],
                [cos(alpha)*d],
                [1]])
    return TM

# RM_2_3 = Homogeneous_transform_matrix(alpha2,a2,d3,theta3).subs(dh)
# TM_2_3 = Translation_matrix(alpha3,a3,d4).subs(dh)
f = Homogeneous_transform_matrix(alpha2,a2,d3,theta3).subs(dh)*Translation_matrix(alpha3,a3,d4).subs(dh)

print("f第一行:\n",f[0])
print("f第二行:\n",f[1])
print("f第三行:\n",f[2])
print("f第四行:\n",f[3])