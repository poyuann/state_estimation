import numpy as np
from casadi import *
import math
import matplotlib.pyplot as plt


def partial_measure(state,gimbal):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 
    h_w = vertcat(dx/dz,dy/dz,dz)

    fx = 960.5885501315905
    fy = 960.5885501315905
    # gimbal = casadi.Opti()
    # yaw = gimbal.variable()
    # pitch = gimbal.variable()
    # obj = get_gimbal(yaw, pitch, h_w)
    # gimbal.minimize(obj)
    # gimbal.solver('ipopt')

    # try:
    #     sol = gimbal.solve()
    #     print("Optimal yaw :", sol.value(yaw))
    #     print("Optimal pitch :", sol.value(pitch))
    #     # print("Optimal t:", sol.value(t))
    # except RuntimeError as e:
    #     print("Solver failed with message:", str(e))
    #     print("Debug values for x:", gimbal.debug.value(yaw))
    # get_R_w2c(sol.value(yaw),sol.value(pitch))
    theta = get_yaw(dx, dy)
    R_w2c = get_R_w2c(gimbal[0], gimbal[1], theta)
    temp = R_w2c @ h_w
    X = temp[0]
    Y = temp[1]
    Z = temp[2]
    H = vertcat(
        horzcat((fx/Z)*(R_w2c[0, 0] - R_w2c[2, 0]*X), (fx/Z)*(R_w2c[0, 1] - R_w2c[2, 1]*X), (fx/Z)*(R_w2c[0, 2] - R_w2c[2, 2]*X)),
        horzcat((fy/Z)*(R_w2c[1, 0] - R_w2c[2, 0]*Y), (fy/Z)*(R_w2c[1, 1] - R_w2c[2, 1]*Y), (fy/Z)*(R_w2c[1, 2] - R_w2c[2, 2]*Y)),
        horzcat(R_w2c[2, 0]                         , R_w2c[2, 1]                         , R_w2c[2, 2] )
    )  
    return H
def get_gimbal(yaw, pitch, h_w):
    R_w2c = get_R_w2c(yaw, pitch)
    print(R_w2c.size())
    print(h_w.size())
    X = R_w2c @ h_w
    return X[0]+ X[1]
def get_yaw(dx, dy):
    
    yaw = atan2(dy, dx)
    return yaw
def get_R_w2c(yaw, pitch, theta):

    R_w2b = vertcat(
        horzcat(cos(theta), -sin(theta), 0),
        horzcat(sin(theta), cos(theta), 0),
        horzcat(0 , 0, 1),
    )
    R_b2m = vertcat(
        horzcat(1, 0, 0),
        horzcat(0, -1, 0),
        horzcat(0, 0, -1)
        )
    R_m2p = vertcat(
        horzcat(cos(yaw) , sin(yaw), 0),
        horzcat(-sin(yaw), cos(yaw), 0),
        horzcat(0        , 0       , 1)
    )
    R_p2t = vertcat(
        horzcat(cos(pitch), 0 , -sin(pitch)),
        horzcat(0         , 1 ,  0         ),
        horzcat(sin(pitch), 0 , cos(pitch) )
    )

    R_t2c = vertcat(
        horzcat(0, 1, 0),
        horzcat(0, 0, 1),
        horzcat(1, 0, 0)
    )
    R_w2c = R_t2c @ R_p2t @ R_m2p @ R_b2m
    return R_w2c
def Distance(state):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 

    return dz
def gimbal_constrian(state, gimbal):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 
    X = dx/dz
    Y = dy/dz
    Z = dz

    return (-sin(gimbal[0])@X - cos(gimbal[0])@Y)    
def gimbal_constrian2(state, gimbal):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 
    X = dx/dz
    Y = dy/dz
    Z = dz

    return (sin(gimbal[1])@cos(gimbal[0])@X - sin(gimbal[1])@sin(gimbal[0])@Y - cos(gimbal[1])@Z)  
def gimbal_constrian3(state, gimbal):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 
    X = dx/dz
    Y = dy/dz
    Z = dz    
    return(cos(gimbal[1])@cos(gimbal[0])@X - cos(gimbal[1])@sin(gimbal[0])@Y + sin(gimbal[1])@Z)
def getH(state,alpha,theta):
    dx = state[0,0] * sin(state[1,0])
    dy = state[0,0] * cos(state[1,0])
    dz = state[2,0] 

    fx = 960.5885501315905
    fy = 960.5885501315905
    H = vertcat(
        horzcat(-fx*dy/(cos(alpha)*state[0,0])**2, fx*dx /(cos(alpha)*state[0,0])**2),
        horzcat(fy*dz*cos(theta)/(cos(alpha)*state[0,0])**2,fy*sin(alpha)*dz/(cos(alpha)*state[0,0])**2),
        horzcat(-cos(state[1,1]-state[1,0]), -sin(state[1,1]-state[1,0])),
        horzcat(sin(state[1,2]-state[1,1])/state[0,2], -cos(state[1,2]-state[1,1])/state[0,2])
    )
    return H
def information_matrix(state,alpha):
    R = np.eye(2) *1e-3
    dx = state[0,0] * sin(state[1,0])
    dy = state[0,0] * cos(state[1,0])
    dz = state[2,0] 


    theta = get_yaw(dx, dy)

    H = getH(state,alpha, theta)
    print(det(R))
    return 1/trace(mtimes(mtimes(H,inv(R)),H.T))
def determinant3x3(matrix):
    a = matrix[0,0]
    b = matrix[0,1]
    c = matrix[0,2]
    d = matrix[1,0]
    e = matrix[1,1]
    f = matrix[1,2]
    g = matrix[2,0]
    h = matrix[2,1]
    i = matrix[2,2]

    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    return determinant

def plot_position_3D(optimal):
    fig = plt.figure()
    target = [0,0,10]
    uavs = []
    for i in range(3):
        uav = horzcat(target[0] + optimal[0,i]*sin(optimal[1,i]) , target[1] + optimal[0,i]*cos(optimal[1,i]) , target[2] + optimal[2,i])
        uavs = vertcat(uavs , uav)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(uavs[:,0], uavs[:,1], uavs[:,2], c = 'r', label='Agent Pose target')
    ax.scatter(target[0], target[1], target[2], c = 'g', marker = 'o', label='target Pose t')
    # ax.scatter(1,1,1, c = 'b', marker = 'o', label='test')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()

    plt.title('target 3D Poses')
    plt.show()
def coordinate(optimal):
    target = [0,0,0]
    uavs = []    
    for i in range(3):
        uav = horzcat(target[0] + optimal[0,i]*sin(optimal[1,i]) , target[1] + optimal[0,i]*cos(optimal[1,i]) , target[2] + optimal[2,i])
        uavs = vertcat(uavs , uav)

    return uavs
def agent_distance(uav1,uav2):
    dx = uav1[0] - uav2[0] 
    dy = uav1[1] - uav2[1]
    dz = uav1[2] - uav2[2]

    return  sqrt(dx**2 + dy**2 + dz**2)
def uvd(state,gimbal):
    dx = state[0] * sin(state[1])
    dy = state[0] * cos(state[1])
    dz = state[2] 
    X = dx/dz
    Y = dy/dz
    Z = dz
    u = -sin(gimbal[0])*X - cos(gimbal[0])*Y
    v = sin(gimbal[1])*cos(gimbal[0])*X - sin(gimbal[1])*sin(gimbal[0])*Y - cos(gimbal[1])*Z
    d = cos(gimbal[1])*cos(gimbal[0])*X - cos(gimbal[1])*sin(gimbal[0])*Y + sin(gimbal[1])*Z
    value = [u,v,d]
    return value    
opti = casadi.Opti()
num = 3

# Define variables within the Opti context
x = opti.variable(3, 3)  # 3 UAVs each with 3 coordinates (distance, alpha, beta)
gimbal = opti.variable()
obj = information_matrix(x,gimbal)
opti.minimize(obj)


uavs = coordinate(x)
for i in range(num):
    # opti.subject_to(x[0, i] > 3.0)
    # opti.subject_to(x[0, i] < 5.0)
    opti.subject_to(x[0,i] == 3)
    opti.subject_to(x[2,i] > 0.0)   
    opti.subject_to(x[2,i] < 3.0)   
opti.subject_to(gimbal < 2 *pi * 50/180)
opti.subject_to(gimbal > 0)

for i in range(num):
    opti.subject_to(x[1, i] < 2* pi)
    opti.subject_to(x[1, i] > 0)
# for i in range(num):
#     opti.subject_to(gimbal_constrian(x[:,i],gimbal[:,i]) == 0.0)
#     opti.subject_to(gimbal_constrian2(x[:,i],gimbal[:,i]) == 0.0)
#     opti.subject_to(gimbal_constrian3(x[:,i],gimbal[:,i]) >= 2.0)
#     opti.subject_to(gimbal_constrian3(x[:,i],gimbal[:,i]) < 10.0)
#     opti.subject_to(gimbal[1, i] < pi*80/180)
#     opti.subject_to(gimbal[1, i] > -pi*20/180)
#     opti.subject_to(gimbal[0, i] < pi)
#     opti.subject_to(gimbal[0, i] > -pi) 
opti.subject_to(x[2,1] == x[2,0])
opti.subject_to(x[2,2] == x[2,1])
opti.subject_to(agent_distance(uavs[0,:],uavs[1,:]) > 1.0)
opti.subject_to(agent_distance(uavs[0,:],uavs[2,:]) > 1.0)
opti.subject_to(agent_distance(uavs[1,:],uavs[2,:]) > 1.0)
# for i in range(num):


# Provide an initial guess for the variables
initial_x = np.array([[3, 1, 2], [2, 3, 2], [1, 3, 2]])
initial_g = np.array([[1,1,2],[0,1,1]])
opti.set_initial(x, initial_x)
# opti.set_initial(gimbal, initial_g)

# Debug initial values
print("Initial x:", initial_x)
# print("Initial t:", initial_t)
debug = True
# while(debug):
opti.solver('ipopt')


try:
    sol = opti.solve()
    print("Optimal x:", sol.value(x))
    print("Optimal gimbal:", sol.value(gimbal))

    plot_position_3D(sol.value(x))
    print("uav : ",coordinate(sol.value(x)))
    # for i in range(3):
    #     print(i," :", uvd(sol.value(x)[:,i],sol.value(gimbal)[:,i]))
    print(information_matrix(sol.value(x),sol.value(gimbal)))
    debug = False
except RuntimeError as e:
    print("Solver failed with message:", str(e))
    print("Debug values for x:", opti.debug.value(x))
    print("Debug values for gimbal:", opti.debug.value(gimbal))
    initial_x = opti.debug.value(x)
    initial_g = opti.debug.value(gimbal)
    opti.set_initial(x, initial_x)
    opti.set_initial(gimbal, initial_g)

