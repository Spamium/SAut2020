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


class Kalman_UUV():
    
    
    #DVL = open('DLV.txt', 'r')
    #IMU = open('IMU.txt', 'r')

    
    # TODO add the pose variables
    # ORI = np.array([[0, 0, 0, 0]])
    # ORIvar = np.array
    
    def __init__(self, initialx, initialy, initialvx, initialvy, dt, accelvarx, accelvary):
        
        self.A = np.array([[1, 0, dt, 0],
                           [0, 0, 1, 0],
                           [0, 1, 0, dt],
                           [0, 0, 0, 1]])
        
        self.B = np.array([[(1/2)*(dt**2), 0],
                           [dt, 0],
                           [0, (1/2)*(dt**2)],
                           [0, dt]])
        
        self.H = np.eye(4)
        
        self.accelvar = np.array([[accelvarx, 0],
                                 [0, accelvary]])
        
        self.X = np.array([[initialx],
                           [initialy],
                           [initialvx],
                           [initialvy]])
        
        self.P = np.eye(4)
        
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
        self.update(xest, varxest)
    
    
    
    def predictORI(self):
        return
    
    def updateORI(self):
        return
    
    
    
    def predictPOS(self):
        
        xest = np.add( np.matmul(self.A, self.X), np.matmul(self.B, self.accel))
        
        # Big matrix multiplication incoming
        # looks menacing but means:
        #    varxest = A*varX*At + B*varaccel*Bt
        varxest = np.add( np.matmul(np.matmul(self.A, self.Xvar), self.A.T), np.matmul(np.matmul(self.B, self.accelvar), self.B.T))
        
        return xest, varxest
    
    
    
    def updatePOS(self, xest, varxest):
        
        # Y is the innovation of our filter
        # getLBL should return a line matrix of positions
        # Multiplying our xest by H will remove the speed and return a line
        #   matrix with the positions from the prediction step 
        measx, measvar = self.getLBL()
        Y = measx - np.matmul(self.H, xest)

        
        # TODO figure out wtf that R is.... it's a variance from somewhere
        Sk = np.add(np.matmul(np.matmul(self.H, self.Xvar), self.H.T), measvar)
        
                
        # K is our Kalman gain
        K = np.matmul(np.matmul(self.Xvar, self.H.T), np.linalg.inv(Sk))
        
        
        self.X = np.add(self.Xest, np.matmul(K, Y)) 
        self.Xvar = np.matmul(np.subtract(np.eye(), np.matmul(K, self.H)), self.Xvar)
        
        return 
    
    
    
    
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
