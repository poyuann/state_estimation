import numpy as np
from casadi import *
import math
import matplotlib.pyplot as plt
def partial_measure(pose1,pose2):
    # pose1 = pose1.reshape(6,1)
    # pose2 = pose2.reshape(6,1)

    dx = (pose2[0] - pose1[0]) 
    dy = (pose2[1] - pose1[1])
    dz = (pose2[2] - pose1[2])
    print("pass")
    r = sqrt(dx**2 + dy**2 + dz**2)

    H_i = np.matrix([[ -dx/r       , -dy/r                         , -dz/r ,0,0,0],
                    [-dz*dx/(r*sqrt(dx**2 + dy**2))  , -dz*dy/(r*sqrt(dx**2 + dy**2))    , sqrt(dx**2 + dy**2)/(r**2),0,0,0],
                    [dy/(dx**2 + dy**2)           , -dx/(dx**2+dy**2)               , 0,0,0,0]])
    return H_i
def Distance(pose1,pose2):
    dx = pose2[0] - pose1[0]
    dy = pose2[1] - pose1[1]
    dz = pose2[2] - pose1[2]

    return sqrt(dx**2 + dy**2 + dz**2)
def information_matrix(robot,target):
    P = np.identity(6)
    R = np.eye(3)
    num = 3
    temp_s = []
    # print(robot)
    for i in range(num):
        inv_matrix = np.linalg.inv(inv(R+ partial_measure(robot[:,i],target)*P*partial_measure(robot[:,i],target).T))
        # temp_s.append(mtimes(mtimes(partial_measure(robot[:,i],target).T  ,inv(R+ partial_measure(robot[:,i],target)*P*partial_measure(robot[:,i],target).T)),-partial_measure(robot[:,i],target)))
    alpha = 0 
    for i in range(num):
        alpha = alpha + np.trace(temp_s[i])
        
    s = np.zeros([6,6])
    for i in range(num):
        s = s + alpha * temp_s[i]
    return s 


x = np.matrix(np.arange(36).reshape((6,6)))
x1 = np.random.randint(15, size=(6,1)) 
array = np.random.randint(15, size=(6,3)) 
print(information_matrix(array,x1))
# opti = casadi.Opti()
# num = 3 
# x = opti.variable(6,num)
# y = opti.variable(6)

# obj = information_matrix(x,y)
# opti.minimize( obj )
# for i in range(num) :
#     for j in range(num) :
#         # opti.subject_to( Distance(x[:,i],x[0:,j]) >= 3 )
#         if  i !=j :
#             opti.subject_to( Distance(x[:,i],x[0:,j]) >= 3 )

#             # opti.subject_to(sqrt((x[0,i]-x[0,j])**2 + (x[1,i]-x[1,j])**2 + (x[2,i]-x[2,j])**2) >= 3)
# for i in range(num):
#     opti.subject_to( Distance(x[:,i],y) == 1 )

# opti.solver('ipopt')


# sol = opti.solve()

# print(sol.value(x))
# print(sol.value(y))

# plt.plot(S)
# plt.xlabel('Diameter')
# plt.ylabel('Cost')
# plt.show()