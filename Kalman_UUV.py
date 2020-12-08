# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 15:57:34 2020

@author: hugo0

Important notes:
    Sensors will be publishing at a set frequency.
    Our goal is to adjust 
    
    Since we have access to position, speed and acceleration discretely specified
    in each axis, and since we can assume our unmaned vehicle to be free floating in water
    making movement in each axis independent of movement in the others, we simplify our problem
    to two Kalman Filter iterations.
    
    We will separately calculate our position and pose:
        - position from LBL and an estimate from linear velocity and acceleration
        - pose from angular velocity and acceleration
    
    If we have no easy way to infer our pose with any sensor, our location will always be
    more accurate than our pose, but due to the way our sensors work that will not be
    an issue for our localization problem.
    
    If it can't be done here, let the programmer in charge of the control module deal
    with it...

"""

import numpy as np
from time import perf_counter



class Kalman_UUV():
    
    
    #DVL = open('DLV.txt', 'r')
    #IMU = open('IMU.txt', 'r')
    lastTime = 0
    
    # X will be our position/speed vector
    # first line represents position in each axis (x, y)
    # second line represents speed in each axis (vx, vy)
    X = np.array([[0,0], [0,0]])
    # accel will be our acceleration vector
    # technically the control variable, since we have no reliable way to get odometry from the thruster
    # accel represents linear acceleration in each axis (ax, ay)
    accel = np.array([0, 0])
    
    # TODO add the pose variables
    
    
    def __init__(self):
        
        # TODO get frequency from ROS
        # TODO deltaT for A/B from frequency
        self.A = np.array([[1, 0], [0, 1]])
        self.B = np.array([[0, 0]]).T
        
        return
    
    
    
    def Kalmancycle(self):
        deltaT = self.getDelta()
        self.lastTime = perf_counter()
        # Set values for A matrix, top-right corner is dt
        self.A[0][1] = deltaT
        # Set values for B column array, top value is half of dt squared, bottom value is simply dt
        self.B[0][0] = (1/2)*deltaT**2
        self.B[1][0] = deltaT
        
        self.getSpeed()
        self.getAccel()
        
        xest, varxest = self.predict()
        
    
    
    
    def predict(self):
        
        xest = np.add( np.matmul(self.A, self.X), np.matmul(self.B, self.accel))
        
        # Big matrix multiplication incoming
        # looks menacing but means:
        #    varxest = A*varX*At + B*varaccel*Bt
        varxest = np.add( np.matmul(np.matmul(self.A, self.Xvar), self.A.T), np.matmul(np.matmul(self.B, self.accelvar), self.B.T))
        
        return xest, varxest
    
    
    
    def matching(self):
        return
    
    
    
    
    
    def getDelta(self):
        return perf_counter() - self.lastTime
    
    # get reading from DVL
    # provisionally, read from file DVL.txt
    def getSpeed(self):
        return self.DVL.readline()
        
        
    # get reading from IMU
    # provisionally, read from file IMU.txt
    def getAccel(self):
        return self.IMU.readline()
    
    def getLBL(self):
        return
    

print(np.diag([1,2,3]))    
