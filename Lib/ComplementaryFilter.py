# Libs.
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import time


class ComplementaryFilter():
    def __init__(self, a1=0.05, a2=0.01):
        # Coefficient Matrix
        diag = [a1, a1, a1, a2, a2, a2]
        self.A1 = np.diag(diag)
        self.A2 = np.eye(6) - self.A1

        # Prev pos and v
        self.prev_v = np.array([[0], [0], [0]])
        self.prev_p = np.array([[0], [0], [0]])

    def fuse(self, p, p_d, a, dt=1/30):
        p_d = np.array(p_d)/dt
        a = np.array(a) - np.array([0,0,9.81])
        # print(a)
        X = np.array([[p[0]], [p[1]], [p[2]], [p_d[0]], [p_d[1]], [p_d[2]]])
        
        Y = np.array([
            [self.prev_p[0] + self.prev_v[0] * dt + 0.5 * a[0] * dt**2], 
            [self.prev_p[1] + self.prev_v[1] * dt + 0.5 * a[1] * dt**2],
            [self.prev_p[2] + self.prev_v[2] * dt + 0.5 * a[2] * dt**2],
            [self.prev_v[0] + a[0] * dt],
            [self.prev_v[1] + a[1] * dt],
            [self.prev_v[2] + a[2] * dt]
        ]).reshape(6, 1)  # Explicitly reshape to (6, 1)

        Z = np.dot(self.A1, X) + np.dot(self.A2, Y)

        self.prev_p = Z[0:3, 0]
        self.prev_v = Z[3:6, 0]

        return Z
    

if __name__ == '__main__':

    cf = ComplementaryFilter()

    p = np.asarray([1,0,0])

    pd = np.asarray([1,0,0])

    a = np.asarray([1,0,0])

    dt = 0.1

    cf.fuse(p,pd,a,dt)