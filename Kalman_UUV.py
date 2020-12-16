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
    
    def __init__(self, posvec, velvec, accelvec, dt, avar):
        
        initialposx = posvec[0]
        initialposy = posvec[1]
        initialtheta = posvec[2]
        
        initialvx = velvec[0]
        initialvy = velvec[1]
        initialomega = velvec[2]
        
        initialax = accelvec[0]
        initialay = accelvec[1]
        initialalpha = accelvec[2]
        
        axvar = avar[0]
        ayvar = avar[1]
        alpha = avar[2]
        
        self.F = np.array([[1, 0, 0, dt, 0, 0],
                           [0, 1, 0, 0, dt, 0],
                           [0, 0, 1, 0, 0, dt, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        
        self.G = np.array([[(dt**2)/2, 0, 0],
                           [0, (dt**2)/2, 0],
                           [0, (dt**2)/2],
                           [dt, 0, 0],
                           [0, dt, 0],
                           [0, 0, dt]])
        # ok estamos a comparar com x,y,theta,vx,vy e ... ? 
        self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0]])
        
        self.Q = np.array([[axvar, 0, 0],
                                  [0, ayvar, 0]
                                  [0, 0, alpha]])
        
        self.X = np.array([[initialposx],
                           [initialposy],
                           [initialtheta],
                           [initialvx],
                           [initialvy],
                           [initialomega],
                           [initialax],
                           [initialay],
                           [initialalpha]])
        
        self.P = np.matmul(np.matmul(self.G, self.Q), self.G.T)
        
        return
    
    
    
    def Kalmancycle(self, posmeas, varpos, velmeas, varvel):
        
        self.predictPOS()
        # TODO get stuff from LBL for update
        self.getLBL()
        self.updatePOS(posmeas, varpos, velmeas, varvel)
    
    def predictPOS(self):
        
        self.X = np.matmul(self.F, self.X)
        
        # Big matrix multiplication incoming
        # looks menacing but means:
        #    varxest = A*varX*At + B*varaccel*Bt
        self.P = np.add( np.matmul(np.matmul(self.F, self.P), self.F.T), np.matmul(np.matmul(self.G, self.Q), self.G.T))
        
        return
    
    
    
    def updatePOS(self, lblmeas, lblvar, dvlmeas, dvlvar, imumeas, imuvar):
        
        # TODO 
        # ON UPDATE WE GET THE VARIANCE FOR THE MEASUREMENTS AS WELL!!!!
        
        posx = lblmeas[0]
        posy = lblmeas[1]
        theta = imumeas[0]
        
        velx = dvlmeas[0]
        vely = dvlmeas[1]
        omega = imumeas[1]
        
        posvarx = lblvar[0]
        posvary = lblvar[1]
        vartheta = imuvar[0]
        
        velvarx = dvlvar[0]
        velvary = dvlvar[1]
        varomega = imuvar[1] 
        
        
        
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
