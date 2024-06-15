import math
from scipy.io import loadmat
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
import numpy as np

class KalmanFilter():
    def __init__(self):    
        # varience system
        self.Q1 = 1 # x 
        self.Q2 = 1 # y
        self.Q3 = 1 # z

        self.Q4 = 1 # vx
        self.Q5 = 1 # vy
        self.Q6 = 1 # vz

        # varience measurement
        self.R1 = 10 # x
        self.R2 = 10 # y
        self.R3 = 10 # z

        self.R4 = 5000 # vx
        self.R5 = 5000 # vy
        self.R6 = 1000 # vz

        self.Q = np.diag(([self.Q1,self.Q2,self.Q3,self.Q4,self.Q5,self.Q6]))
        self.R = np.diag(([self.R1,self.R2,self.R3,self.R4,self.R5,self.R6]))

        self.X_post = np.zeros(6)

        self.P_post = np.diag(np.ones(6))
                
    
        
    def fuse(self, p, p_d, a, dt=1/30):

        # vel m/s
        p_d = np.array(p_d)/dt
        # acc m/s2
        a = np.array(a) - np.array([0,0,9.81])

        # system model
        A = [[1,0,0,dt,0,0],
             [0,1,0,0,dt,0],
             [0,0,1,0,0,dt],
             [0,0,0,1,0,0],
             [0,0,0,0,1,0],
             [0,0,0,0,0,1]
        ]
        # input model
        B = [[0.5*dt**2,0,0],
             [0,0.5*dt**2,0],
             [0,0,0.5*dt**2],
             [dt,0,0],
             [0,dt,0],
             [0,0,dt]]



        # measurement model
        H = np.diag(np.ones(6))


        # measurements
        Y = np.concatenate((p, p_d), axis=None)

        X_Pre = np.matmul(A,self.X_post) + np.matmul(B,a)
        P_Pre = np.matmul(A,np.matmul(self.P_post,np.transpose(A))) + self.Q

        ## kalman gain
        #  Z and VZ  Kalman Gains
        D = np.matmul(H,np.matmul(P_Pre,np.transpose(H))) + self.R
        K = np.matmul(P_Pre,np.matmul(np.transpose(H),np.linalg.inv(D)))
        # print(K[3][3]) # visualize gain
        ## Updates
        self.X_post = X_Pre + np.matmul(K,np.subtract(Y,X_Pre))
        self.P_post = np.matmul(np.eye(6) - np.matmul(K,H),P_Pre)

        return self.X_post 


if __name__ == '__main__':

    kf = KalmanFilter()

    p = np.asarray([1,0,0])

    pd = np.asarray([1,0,0])

    a = np.asarray([1,0,0])

    dt = 0.1

    kf.fuse(p,pd,a,dt)
    kf.fuse(p,pd,a,dt)
    kf.fuse(p,pd,a,dt)
  