# Trajectory

import numpy as np
from scipy import linalg

def p2pTrjTrap1DOF(t,T,Pos,c):

    n = T.size - 1 # n = number of trajectories

    for j in range(n,0,-1):
        if t <= T[:,j]:
            i = j-1

    qi = Pos[i]
    qf = Pos[i+1]
    ti = t-T[:,i]
    tf = T[:,i+1]-T[:,i]
    
    D = qf - qi
    signD = np.sign(D)

    ta = tf/(2+c)
    kv = abs(D)/((c+1)*ta)
    ka = abs(D)/((c+1)*ta**2)
    
    if ti <= ta:
        q_pos = qi + 0.5*ti**2*ka*signD
    elif ti <= tf - ta:
        q_pos = qi + (ti-0.5*ta)*kv*signD
    else:
        q_pos = qf - 0.5*(tf-ti)**2*ka*signD

    return q_pos

# def minimumJerkConstrCoefficient1DOF(T,Pos,Vel,Acc):

#     n = T.size - 1 # n = number of trajectories 

#     # Waypoints' Matrix
#     R = np.zeros((6*n,6*n)) # Start with zero matrix for R, joint

#     # Positions of all waypoints
#     for i in range(n):
#         # Position
#         R[2*i+0,6*i-6:6*i] = np.array([pow(T[i],5),pow(T[i],4),pow(T[i],3),pow(T[i],2),T[i],1])
#         R[2*i+1,6*i-6:6*i] = np.array([pow(T[i+1],5),pow(T[i+1],4),pow(T[i+1],3),pow(T[i+1],2),T[i+1],1])
#         # Velocity
#         R[2*n+2*i+0,6*i-6:6*i] = np.array([5*pow(T[i],4),4*pow(T[i],3),3*pow(T[i],2),2*T[i],1,0])
#         R[2*n+2*i+1,6*i-6:6*i] = np.array([5*pow(T[i+1],4),4*pow(T[i+1],3),3*pow(T[i+1],2),2*T[i+1],1,0])
#         # Accelaration
#         R[4*n+2*i+0,6*i-6:6*i] = np.array([20*pow(T[i],3),12*pow(T[i],2),6*T[i],2,0,0])
#         R[4*n+2*i+1,6*i-6:6*i] = np.array([20*pow(T[i+1],3),12*pow(T[i+1],2),6*T[i+1],2,0,0])

#     # Boundary Conditions Matrix
#     BC = np.zeros((6*n,1))
#     BC(1,1) = Pos(:,1);      	# Position of the first waypoint
#     BC(1+2*n:1+2*n) = Vel(:,1);	# Velocity of the first waypoint
#     BC(1+4*n:1+4*n) = Acc(:,1);	# Acceleration of the first waypoint
#     if n > 1
#         PosInter = zeros(2*(n-1),1);
#         VelInter = zeros(2*(n-1),1);
#         AccInter = zeros(2*(n-1),1);
#     end
#     for i = 2:n
#         PosInter(2*i-3:2*i-2,:) = [Pos(:,i)';Pos(:,i)'];
#         VelInter(2*i-3:2*i-2,:) = [Vel(:,i)';Vel(:,i)'];
#         AccInter(2*i-3:2*i-2,:) = [Acc(:,i)';Acc(:,i)'];
#     end
#     for i = 1:2*(n-1)
#         # Position
#         BC(1+i:i+1) = PosInter(i,:);
#         # Velocity
#         BC(1+2*n+i:2*n+i+1) = VelInter(i,:);
#         # Acceelration
#         BC(1+4*n+i:4*n+i+1) = AccInter(i,:);
#     end
#     BC(1+2*n-1:2*n+0) = Pos(:,n+1); % Position of the final waypoint
#     BC(1+2*n+1:2*n+2) = Vel(:,n+1); % Velocity of the final waypoint
#     BC(1+2*n+3:2*n+4) = Acc(:,n+1); % Acceleration of the final waypoint

#     # Coefficient  Vector
#     Cj = linalg.solve(R, BC) # Cj = R\BC;
    
#     return Cj
    
def minimumJerkCoefficient1DOF(T,Pos,Vel,Acc):
    
    n = T.size - 1 # n = number of trajectories 

    # Waypoints' Matrix
    R = np.zeros((6*n,6*n)) # Start with zero matrix for R, joint

    # Positions of all waypoints
    for i in range(n):
        R[0+2*i,6*i:6+6*i] = np.array([pow(T[i],5),pow(T[i],4),pow(T[i],3),pow(T[i],2),T[i],1])
        R[1+2*i,6*i:6+6*i] = np.array([pow(T[i+1],5),pow(T[i+1],4),pow(T[i+1],3),pow(T[i+1],2),T[i+1],1])
    # Velocity boundary conditions (inital and final waypoints)        
    R[2*n,0:6]              = np.array([5*pow(T[0],4),4*pow(T[0],3),3*pow(T[0],2),2*T[0],1,0])
    R[1+2*n,6*(n-1):1+6*n]  = np.array([5*pow(T[n],4),4*pow(T[n],3),3*pow(T[n],2),2*T[n],1,0])
    # Equal Accelaration boundary conditions (initial and final waypoints)
    R[2+2*n,0:6]            = np.array([20*pow(T[0],3),12*pow(T[0],2),6*T[0],2,0,0])
    R[3+2*n,6*(n-1):1+6*n]  = np.array([20*pow(T[n],3),12*pow(T[n],2),6*T[n],2,0,0])
    #Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i in range(n-1):
        R[i+0*(n-1)+4+2*n,6*i:6*(i+2)] = np.array([ 5*pow(T[i+1],4), 4*pow(T[i+1],3),3*pow(T[i+1],2),2*T[i+1],1,0, -5*pow(T[i+1],4), -4*pow(T[i+1],3),-3*pow(T[i+1],2),-2*T[i+1],-1,0]) # Equal velocity at intermediate waypoints
        R[i+1*(n-1)+4+2*n,6*i:6*(i+2)] = np.array([20*pow(T[i+1],3),12*pow(T[i+1],2),6*T[i+1]       ,2       ,0,0,-20*pow(T[i+1],3),-12*pow(T[i+1],2),-6*T[i+1]       ,-2       , 0,0]) # Equal acceleration at intermediate waypoints
        R[i+2*(n-1)+4+2*n,6*i:6*(i+2)] = np.array([60*pow(T[i+1],2),24*T[i+1]       ,6              ,0       ,0,0,-60*pow(T[i+1],2),-24*T[i+1]       ,-6              ,0        , 0,0]) # Equal jerk at intermediate waypoints
        R[i+3*(n-1)+4+2*n,6*i:6*(i+2)] = np.array([120*T[i+1]      ,24              ,0              ,0       ,0,0,-120*T[i+1]      ,-24              ,0               ,0        , 0,0]) # Equal snap at intermediate waypoints

    # Boundary Conditions Matrix
    BC = np.zeros((6*n,1))
    BC[0,0] = Pos[0]    # Position of the first waypoint
    if n > 0:
        PosInter = np.zeros((2*(n-1),1))
    for i in range(1,n):
        PosInter[2*i-2] = Pos[i]
        PosInter[2*i-1] = Pos[i]
    for i in range(2*(n-1)):
        BC[i+1] = PosInter[i]
    BC[2*(n-1)+1] = Pos[n]  # Position of the final waypoint
    BC[2*(n-1)+2] = Vel[0]  # initial velocity
    BC[2*(n-1)+3] = Vel[1]  # final velocity
    BC[2*(n-1)+4] = Acc[0]  # initial acceleration
    BC[2*(n-1)+5] = Acc[1]  # final acceleration
    
    # Coefficient  Vector
    Cj = linalg.solve(R, BC) # Cj = R\BC;
    
    return Cj

def minimumJerkCoefficient(T,Pos,Vel,Acc):
    
    d = Pos.shape[0]
    p = Pos.shape[1]
    n = p-1 # n = number of trajectories 

    I = np.identity(d)
    Z = np.zeros((d,d))

    # Waypoints' Matrix
    R = np.zeros((6*d*n,6*d*n)) # Start with zero matrix for R, joint
    # Positions of all waypoints
    T_0 = np.squeeze(T[:,0])
    T_n = np.squeeze(T[:,n])
    for i in range(n):
        T_i = np.squeeze(T[:,i])
        T_j = np.squeeze(T[:,i+1])
        R[d*(0+2*i):d*(1+2*i),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([I*pow(T_i,5),I*pow(T_i,4),I*pow(T_i,3),I*pow(T_i,2),I*T_i,I]),(1,0,2)),(d,6*d))
        R[d*(1+2*i):d*(2+2*i),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([I*pow(T_j,5),I*pow(T_j,4),I*pow(T_j,3),I*pow(T_j,2),I*T_j,I]),(1,0,2)),(d,6*d))
    # Velocity boundary conditions (inital and final waypoints)
    R[d*(0+2*n):d*(1+2*n),        0:6*d  ] = np.reshape(np.transpose(np.array([5*I*pow(T_0,4),4*I*pow(T_0,3),3*I*pow(T_0,2),2*I*T_0,I,Z]),(1,0,2)),(d,6*d))
    R[d*(1+2*n):d*(2+2*n),6*d*(n-1):6*d*n] = np.reshape(np.transpose(np.array([5*I*pow(T_n,4),4*I*pow(T_n,3),3*I*pow(T_n,2),2*I*T_n,I,Z]),(1,0,2)),(d,6*d))
    # Equal Accelaration boundary conditions (initial and final waypoints)
    R[d*(2+2*n):d*(3+2*n),        0:6*d  ] = np.reshape(np.transpose(np.array([20*I*pow(T_0,3),12*I*pow(T_0,2),6*I*T_0,2*I,Z,Z]),(1,0,2)),(d,6*d))
    R[d*(3+2*n):d*(4+2*n),6*d*(n-1):6*d*n] = np.reshape(np.transpose(np.array([20*I*pow(T_n,3),12*I*pow(T_n,2),6*I*T_n,2*I,Z,Z]),(1,0,2)),(d,6*d))
    #Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i in range(n-1):
        T_j = np.squeeze(T[:,i+1])
        R[d*(4+i+0*(n-1)+2*n):d*(5+i+0*(n-1)+2*n),6*d*i:6*d*(i+2)] = np.reshape(np.transpose(np.array([  5*I*pow(T_j,4), 4*I*pow(T_j,3),3*I*pow(T_j,2),2*I*T_j,I,Z,  -5*I*pow(T_j,4), -4*I*pow(T_j,3),-3*I*pow(T_j,2),-2*I*T_j,-I,Z]),(1,0,2)),(d,6*d*n)) # Equal velocity at intermediate waypoints
        R[d*(4+i+1*(n-1)+2*n):d*(5+i+1*(n-1)+2*n),6*d*i:6*d*(i+2)] = np.reshape(np.transpose(np.array([ 20*I*pow(T_j,3),12*I*pow(T_j,2),6*I*T_j       ,2*I    ,Z,Z, -20*I*pow(T_j,3),-12*I*pow(T_j,2),-6*I*T_j       ,-2*I    , Z,Z]),(1,0,2)),(d,6*d*n)) # Equal acceleration at intermediate waypoints
        R[d*(4+i+2*(n-1)+2*n):d*(5+i+2*(n-1)+2*n),6*d*i:6*d*(i+2)] = np.reshape(np.transpose(np.array([ 60*I*pow(T_j,2),24*I*T_j       ,6*I           ,Z      ,Z,Z, -60*I*pow(T_j,2),-24*I*T_j       ,-6*I           ,Z       , Z,Z]),(1,0,2)),(d,6*d*n)) # Equal jerk at intermediate waypoints
        R[d*(4+i+3*(n-1)+2*n):d*(5+i+3*(n-1)+2*n),6*d*i:6*d*(i+2)] = np.reshape(np.transpose(np.array([120*I*T_j       ,24*I           ,Z             ,Z      ,Z,Z,-120*I*T_j       ,-24*I           ,Z              ,Z       , Z,Z]),(1,0,2)),(d,6*d*n)) # Equal snap at intermediate waypoints

    # Boundary Conditions Matrix
    if n > 1:
        PosInter = np.zeros((2*(n-1),d))
    for i in range(1,n):
        PosInter[2*i-2,:] = np.transpose(Pos[:,i])
        PosInter[2*i-1,:] = np.transpose(Pos[:,i])
    BC = np.zeros((6*d*n,1))
    BC[0:d,0] = Pos[:,0]    # Position of the first waypoint
    for i in range(2*(n-1)):
        BC[d*(i+1):d*(i+2),0] = PosInter[i,:]
    BC[d*(1+2*(n-1)):d*(2+2*(n-1)),0] = Pos[:,n]  # Position of the final waypoint
    BC[d*(2+2*(n-1)):d*(3+2*(n-1)),0] = Vel[:,0]  # initial velocity
    BC[d*(3+2*(n-1)):d*(4+2*(n-1)),0] = Vel[:,1]  # final velocity
    BC[d*(4+2*(n-1)):d*(5+2*(n-1)),0] = Acc[:,0]  # initial acceleration
    BC[d*(5+2*(n-1)):d*(6+2*(n-1)),0] = Acc[:,1]  # final acceleration
 
    # Coefficient  Vector
    Cj = linalg.solve(R, BC) # Cj = R\BC;
    
    return Cj

def minimumJerkConstrCoefficient(T,Pos,Vel,Acc):

    d = Pos.shape[0] # Number of dimensions
    p = Pos.shape[1] # Number of waypoints
    n = p-1 # Number of trajectories 

    I = np.identity(d)
    Z = np.zeros((d,d))

    # Waypoints' Matrix
    R = np.zeros((6*d*n,6*d*n)) # Start with zero matrix for R, joint
    # Positions of all waypoints
    for i in range(n):
        T_i = np.squeeze(T[:,i])
        T_j = np.squeeze(T[:,i+1])
        # Position
        R[d*(0+2*i):d*(1+2*i),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([I*pow(T_i,5),I*pow(T_i,4),I*pow(T_i,3),I*pow(T_i,2),I*T_i,I]),(1,0,2)),(d,6*d))
        R[d*(1+2*i):d*(2+2*i),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([I*pow(T_j,5),I*pow(T_j,4),I*pow(T_j,3),I*pow(T_j,2),I*T_j,I]),(1,0,2)),(d,6*d))
        # Velocity
        R[d*(0+2*i+2*n):d*(1+2*i+2*n),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([5*I*pow(T_i,4),4*I*pow(T_i,3),3*I*pow(T_i,2),2*I*T_i,I,Z]),(1,0,2)),(d,6*d))
        R[d*(1+2*i+2*n):d*(2+2*i+2*n),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([5*I*pow(T_j,4),4*I*pow(T_j,3),3*I*pow(T_j,2),2*I*T_j,I,Z]),(1,0,2)),(d,6*d))
        # Accelaration
        R[d*(0+2*i+4*n):d*(1+2*i+4*n),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([20*I*pow(T_i,3),12*I*pow(T_i,2),6*I*T_i,2*I,Z,Z]),(1,0,2)),(d,6*d))
        R[d*(1+2*i+4*n):d*(2+2*i+4*n),6*d*i:6*d*(1+i)] = np.reshape(np.transpose(np.array([20*I*pow(T_j,3),12*I*pow(T_j,2),6*I*T_j,2*I,Z,Z]),(1,0,2)),(d,6*d))

    # Boundary Conditions Matrix
    if n > 1:
        PosInter = np.zeros((2*(n-1),d))
        VelInter = np.zeros((2*(n-1),d))
        AccInter = np.zeros((2*(n-1),d))
    for i in range(1,n):
        PosInter[2*i-2,:] = np.transpose(Pos[:,i])
        PosInter[2*i-1,:] = np.transpose(Pos[:,i])
        VelInter[2*i-2,:] = np.transpose(Vel[:,i])
        VelInter[2*i-1,:] = np.transpose(Vel[:,i])
        AccInter[2*i-2,:] = np.transpose(Acc[:,i])
        AccInter[2*i-1,:] = np.transpose(Acc[:,i])
    BC = np.zeros((6*d*n,1))
    BC[0:d,0] = Pos[:,0]            # Position of the first waypoint
    BC[2*d*n:d*(1+2*n),0] = Vel[:,0]  # Velocity of the first waypoint
    BC[4*d*n:d*(1+4*n),0] = Acc[:,0]	# Acceleration of the first waypoint
    for i in range(2*(n-1)):
        # Position
        BC[d*(i+1):d*(i+2),0] = PosInter[i,:]
        # Velocity
        BC[d*(i+1+2*n):d*(i+2+2*n),0] = VelInter[i,:]
        # Acceelration
        BC[d*(i+1+4*n):d*(i+2+4*n),0] = AccInter[i,:]
    BC[d*(2*n-1):d*(2*n+0),0] = Pos[:,n] # Position of the final waypoint
    BC[d*(2*n+1):d*(2*n+2),0] = Vel[:,n] # Velocity of the final waypoint
    BC[d*(2*n+3):d*(2*n+4),0] = Acc[:,n] # Acceleration of the final waypoint

    # Coefficient  Vector
    Cj = linalg.solve(R, BC) # Cj = R\BC;
    
    return Cj
    
def minimumJerkPolynomial1DOF(t,T,Cj):

    n = T.size - 1 # n = number of trajectories
    
    for j in range(n,0,-1):
        if t <= T[j]:
            i = j-1
    
    q_pos = Cj[6*i]*pow(t,5) + Cj[1+6*i]*pow(t,4) + Cj[2+6*i]*pow(t,3) + Cj[3+6*i]*pow(t,2) + Cj[4+6*i]*t + Cj[5+6*i]

    return q_pos

def minimumJerkPolynomial(t,T,Cj):

    n = T.size - 1 # n = number of trajectories
    d = round(Cj.shape[0]/(6*n))
    
    for j in range(n,0,-1):
        if t <= T[:,j]:
            i = j-1
    
    out = Cj[d*(6*i+0):d*(6*i+1)]*pow(t,5) + Cj[d*(6*i+1):d*(6*i+2)]*pow(t,4) + Cj[d*(6*i+2):d*(6*i+3)]*pow(t,3) + Cj[d*(6*i+3):d*(6*i+4)]*pow(t,2) + Cj[d*(6*i+4):d*(6*i+5)]*t + Cj[d*(6*i+5):d*(6*i+6)]

    return out
