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
    
    def __init__(self, initialpos, initialv, dt, accelvar):
        
        self.A = np.array([[1, dt],
                           [0, 1]])
        
        self.B = np.array([[(1/2)*(dt**2)],
                           [dt]])
        
        self.H = np.eye(2)
        
        self.accelvar = accelvar
        
        self.X = np.array([[initialpos],
                           [initialv]])
        
        self.P = np.eye(2)
        
        return
    
    
    
    def Kalmancycle(self, posmeas, varpos, velmeas, varvel):
        
        self.predictPOS()
        # TODO get stuff from LBL for update
        self.getLBL()
        self.updatePOS(posmeas, varpos, velmeas, varvel)
    
    
    
    def predictORI(self):
        return
    
    def updateORI(self):
        return
    
    
    
    def predictPOS(self):
        
        self.X = np.matmul(self.A, self.X)
        
        # Big matrix multiplication incoming
        # looks menacing but means:
        #    varxest = A*varX*At + B*varaccel*Bt
        self.P = np.add( np.matmul(np.matmul(self.A, self.P), self.A.T), np.matmul(np.matmul(self.B, self.accelvar), self.B.T))
        
        return
    
    
    
    def updatePOS(self, posmeas, varpos, velmeas, varvel):
        
        # Y is the innovation of our filter
        Y = posmeas - self.X

        R = np.array([[varpos, 0],
                      [0, varvel]])
        # TODO figure out wtf that R is.... it's a variance from somewhere
        Sk = np.add( self.P, R)
        
                
        # K is our Kalman gain
        K = np.matmul(self.Xvar, np.linalg.inv(Sk))
        
        
        self.X = np.add(self.X, np.matmul(K, Y)) 
        self.P = np.matmul(np.subtract(np.eye(), K), self.P)
        
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
