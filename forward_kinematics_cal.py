#/usr/bin/python
import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

# 声明变量以供数学表达式
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')                                 # joint angles theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')                                 # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')                                 # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # joint twist angles

# DH Table
dh = {alpha0:      0, a0:      0, d1:  0.75, theta1:        theta1,
      alpha1:  -pi/2, a1:   0.35, d2:  0.00, theta2: -pi/2.+theta2,
      alpha2:      0, a2:   1.25, d3:     0, theta3:        theta3,
      alpha3: -pi/2., a3: -0.054, d4:  1.50, theta4:        theta4,
      alpha4:  pi/2., a4:      0, d5:  0.00, theta5:        theta5,
      alpha5: -pi/2., a5:      0, d6: 0.193, theta6:        theta6,
      alpha6:      0, a6:      0, d7: 0.110, theta7:         0}

# DH Table
dh2 = {alpha0:      0, a0:      0, d1:  0.75, theta1:        theta1,
      alpha1: -pi/2., a1:   0.35, d2:     0, theta2: -pi/2.+theta2,
      alpha2:      0, a2:   1.25, d3:     0, theta3:        theta3,
      alpha3: -pi/2., a3: -0.054, d4:   1.5, theta4:        theta4,
      alpha4:  pi/2., a4:      0, d5:     0, theta5:        theta5,
      alpha5: -pi/2., a5:      0, d6:     0, theta6:        theta6,
      alpha6:      0, a6:      0, d7: 0.303, theta7:         0}

def transform_matrix(alpha,a,d,theta):
    TM = Matrix([[           cos(theta),           -sin(theta),           0,            a],
                 [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                 [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [                    0,                     0,          0,            1]])
    return TM


## subs的作用是用DH表的具体数值代替
T0_1 = transform_matrix(alpha0, a0, d1, theta1).subs(dh)
T1_2 = transform_matrix(alpha1, a1, d2, theta2).subs(dh)

temp_T0_1 = transform_matrix(alpha0, a0, d1, theta1).subs(dh2)
temp_T1_2 = transform_matrix(alpha1, a1, d2, theta2).subs(dh2)
temp_T2_3 = transform_matrix(alpha2, a2, d3, theta3).subs(dh2)
temp_T3_4 = transform_matrix(alpha3, a3, d4, theta4).subs(dh2)
temp_T4_5 = transform_matrix(alpha4, a4, d5, theta5).subs(dh2)
temp_T5_6 = transform_matrix(alpha5, a5, d6, theta6).subs(dh2)
temp_T6_7 = transform_matrix(alpha6, a6, d7, theta7).subs(dh2)

T2_3 = transform_matrix(alpha2, a2, d3, theta3).subs(dh)
T3_4 = transform_matrix(alpha3, a3, d4, theta4).subs(dh)
T4_5 = transform_matrix(alpha4, a4, d5, theta5).subs(dh)
T5_6 = transform_matrix(alpha5, a5, d6, theta6).subs(dh)
T6_7 = transform_matrix(alpha6, a6, d7, theta7).subs(dh)

temp_T0_2 = (temp_T0_1*temp_T1_2)
temp_T0_3 = (temp_T0_2*temp_T2_3)
temp_T0_4 = (temp_T0_3*temp_T3_4)
temp_T0_5 = (temp_T0_4*temp_T4_5)
temp_T0_6 = (temp_T0_5*temp_T5_6)
temp_T0_7 = (temp_T0_6*temp_T6_7)

T0_2 = (T0_1 * T1_2) ## (Base) Link_0 to Link_2
T0_3 = (T0_2 * T2_3) ## (Base) Link_0 to Link_3
T0_4 = (T0_3 * T3_4) ## (Base) Link_0 to Link_4
T0_5 = (T0_4 * T4_5) ## (Base) Link_0 to Link_5
T0_6 = (T0_5 * T5_6) ## (Base) Link_0 to Link_6 
T0_7 = (T0_6 * T6_7) ## (Base) Link_0 to Link_7 (End Effector)

t01 = T0_1.evalf(subs={theta1:0})**-1
# t01 = T0_1.subs(theta1:0)
t02 = temp_T0_2.evalf(subs={theta1:0,theta2:0})

# inv = np.linalg.inv(t01)
result2 = np.dot(t01,t02)
# temp = np.linalg.solve(t01,t02)

result = T0_7.evalf(subs={theta1:0,theta2:0,theta3:0,theta4:0,theta5:0,theta6:0})

temp_result = temp_T0_7.evalf(subs={theta1:0,theta2:0,theta3:0,theta4:0,theta5:0,theta6:0})

# print("我的DH表计算出的T01\n", T0_1.evalf(subs={theta1:0}))
# print("我的DH表计算出的T12\n", T1_2.evalf(subs={theta1:0,theta2:0,theta3:0,theta4:0,theta5:0,theta6:0}))
# print("他们的DH表计算出的T12\n", temp_T1_2.evalf(subs={theta1:0,theta2:0}))
# print("我的DH表计算出的T02\n", T0_2.evalf(subs={theta1:0,theta2:0}))
# print("他们的DH表计算出的T02\n", temp_T0_2.evalf(subs={theta1:0,theta2:0}))
# print("应该的T_12\n",result2)
print("总:\n",result)
print("temp:\n",temp_result)