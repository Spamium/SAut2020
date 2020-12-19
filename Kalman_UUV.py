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
    
    LBL = open('LBL', 'r')
    LBLvar = open('LBLvar', 'r')
    DVL = open('DLV.txt', 'r')
    DVLvar = open('DLVvar.txt', 'r')
    IMU = open('IMU.txt', 'r')
    IMUvar = open('IMUvar.txt', 'r')

    
    # TODO add the pose variables
    # ORI = np.array([[0, 0, 0, 0]])
    # ORIvar = np.array
    
    def __init__(self):
        
        posvec = self.get_pos()
        velvec = self.get_speed()
        accelvec = self.get_accel()
        dt = self.get_dt()
        avar = self.get_accelvar()
        
        
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
        # X = [x, y, theta, velx, vely, omega, ax, ay, alpha]
        # Numero de linhas em H é o número de variáveis que medimos:
        #   - x, y, theta, velx, vely, omega
        # Numero de colunas em H é o número de componentes em X
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        
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
    
    
    
    def Kalmancycle(self):
        
        i = 0
        # TODO REMOVE THIS
        while(i < 10):
            self.predictPOS()
    
            self.updatePOS()
            
            i += 1
            
        return
    
    def predictPOS(self):
        
        self.X = np.matmul(self.F, self.X)
        
        # Big matrix multiplication incoming
        # looks menacing but means:
        #    varxest = A*varX*At + B*varaccel*Bt
        self.P = np.add( np.matmul(np.matmul(self.F, self.P), self.F.T), np.matmul(np.matmul(self.G, self.Q), self.G.T))
        
        return
    
    
    
    def updatePOS(self):
        
        # TODO 
        # ON UPDATE WE GET THE VARIANCE FOR THE MEASUREMENTS AS WELL!!!!
        
        posmeas = self.get_pos()
        velmeas = self.get_speed()
        posvar = self.get_posvar()
        velvar = self.get_velvar()
        
        
        
        posx = posmeas[0]
        posy = posmeas[1]
        theta = posmeas[2]
        
        velx = velmeas[0]
        vely = velmeas[1]
        omega = velmeas[2]
        
        posvarx = posvar[0]
        posvary = posvar[1]
        vartheta = posvar[2]
        
        velvarx = velvar[0]
        velvary = velvar[1]
        varomega = velvar[2] 
        
        # Vector medido
        meas = np.array([[posx],
                         [posy],
                         [theta],
                         [velx],
                         [vely],
                         [omega]])
        
        # Y is the innovation of our filter
        Y = np.subtract(meas, self.X)
        
        # V é o conjunto das variancias arrumadas na mesma ordem que o vector medido
        V = np.array([[posvarx, 0, 0, 0, 0, 0],
                      [0, posvary, 0, 0, 0, 0],
                      [0, 0, vartheta, 0, 0, 0],
                      [0, 0, 0, velvarx, 0, 0],
                      [0, 0, 0, 0, velvary, 0],
                      [0, 0, 0, 0, 0, varomega]])

        # R = H * H transposed * V      
        R = np.matmul(np.matmul(self.H, self.H.T), V)
        
        # Sk = H * P * H transposed + R
        Sk = np.add( np.matmul(np.matmul(self.H, self.P), self.H.T), R)
                
        # K is our Kalman gain
        # K = P * H transposed * Sk inverse
        K = np.matmul(np.matmul(self.P, self.H.T), np.linalg.inv(Sk))
        
        # X = X-1 + K * Y
        self.X = np.add(self.X, np.matmul(K, Y)) 
        # P = (I - K * H) * P-1
        self.P = np.matmul(np.subtract(np.eye(3), K), self.P)
        
        return 
    
    
    def get_pos(self):
        position = self.LBL.readline().split(" ")
        theta = self.IMU.readline().split(" ")[0]
        
        return [position[0], position[1], theta]
    
    # get reading from DVL
    # provisionally, read from file DVL.txt
    def get_speed(self):
        speed = self.DVL.readline().split(" ")
        omega = self.IMU.readline().split(" ")[3]
        
        return [speed[0], speed[1], omega]
    
    def get_posvar(self):
        position = self.LBL.readline().split(" ")
        theta = self.IMU.readline()
        
        return [position[0], position[1], theta]

    def get_speedvar(self):
        speed = self.DVL.readline().split(" ")
        omega = self.IMU.readline().split(" ")
        
        return [speed[0], speed[1], omega]
    
    # get reading from IMU
    # provisionally, read from file IMU.txt
    def get_accel(self):
        accel = self.IMU.readline().split(" ")

        return [accel[0], accel[1], accel[2]]
        
    def get_accelvar(self):
        accel = self.IMU.readline().split(" ")

        return [accel[0], accel[1]]
        
    
    def get_dt(self):
        return 0.1

filter = Kalman_UUV()
filter.Kalmancycle()