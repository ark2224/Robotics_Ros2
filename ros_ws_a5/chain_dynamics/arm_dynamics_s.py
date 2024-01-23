from arm_dynamics_base import ArmDynamicsBase
import numpy as np
from geometry import rot, xaxis, yaxis


class ArmDynamicsStudent(ArmDynamicsBase):

    def dynamics_step(self, state, action, dt):
        n = self.num_links
        m = self.link_masses
        l = self.link_lengths
        g = 9.8
        phi = self.joint_viscous_friction
        
        iRi1_mats = np.zeros(shape=(n,2,2))
        iRw_mats = np.zeros(shape=(n,2,2))
        iRw_mats[0,:,:] = np.array([[np.cos(state[0]), -np.sin(state[0])], [np.sin(state[0]), np.cos(state[0])]]).T
        omegas = np.zeros((n))
        omegas[0] = state[n]
        for i in range(1,n):
            qi = state[i][0]
            iRi1_mats[i-1,:,:] = np.array([[np.cos(qi), -np.sin(qi)], [np.sin(qi), np.cos(qi)]])
            iRw_mats[i,:,:] = np.matmul(iRi1_mats[i-1,:,:].T, iRw_mats[i-1,:,:])
            omegas[i] = omegas[i-1] + state[n+i]
            
        # setting up matrices for linear set of equations
        A = np.zeros((6*n, 6*n))
        b = np.zeros((6*n, 1))
        for i in range(n):
            # Constraint (1)+(2)
            A[2*i, 2*i] = -1
            A[2*i+1, 2*i+1] = -1
            if i != n-1:
                A[2*i:2*i+2, 2*i+2:2*i+4] = iRi1_mats[i,:,:].reshape((2,2))
            A[2*i, 2*i+2*n] = m[i]
            A[2*i+1, 2*i+2*n+1] = m[i]
            A[2*i+1, 4*n+i] = 0.5*m[i]*l[i]
            b[2*i,0] = m[i]*(0.5*l[i]*(omegas[i]**2) - g*iRw_mats[i,0,1])
            b[2*i+1,0] = -m[i]*g*iRw_mats[i,1,1]
                
            # Constraint (3)
            A[i+2*n, 2*i+1] = -0.5*l[i]
            if i != n-1:
                A[i+2*n, 2*i+2:2*i+4] = -0.5*l[i]*iRi1_mats[i,1,:].reshape((1,2))
            A[i+2*n, i+4*n] = -m[i]*(l[i]**2)/12
            b[i+2*n,0] = state[n+i]*phi - action[i]
            if i != n-1:
                b[i+2*n,0] += action[i+1]
            
            # Constraint (4)
            A[2*i+3*n, 2*n+2*i] = -1
            A[2*i+3*n+1, 2*n+2*i+1] = -1
            if i != 0:
                iRi_1 = np.transpose(iRi1_mats[i-1,:,:].reshape((2,2)))
                A[2*i+3*n:2*i+3*n+2, 2*n+2*(i-1):2*n+2*i] = iRi_1
                A[2*i+3*n:2*i+3*n+2, i+4*n-1] = l[i-1]*iRi_1[:,1]
                b[2*i+3*n:2*i+3*n+2,0] = l[i-1]*(omegas[i-1]**2)*iRi_1[:,0]

            # Constraint (5)
            A[5*n+i, 4*n+i] = -1
            A[5*n+i, 5*n+i] = 1
            if i != 0:
                A[5*n+i, 4*n+i-1] = 1
                
        x = np.linalg.solve(A, b)
        
        # ~~~Euler Integration~~~
        qdot_i_t1 = state[n:] + dt*x[5*n:6*n]
        q_i_t1 = state[:n] + dt*state[n:] + 0.5*(dt**2)*x[5*n:6*n]
        
        return np.concatenate((q_i_t1,qdot_i_t1))

