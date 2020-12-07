# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 15:57:34 2020

@author: hugo0
"""

class Kalman_UUV():
    
    
    DVL = open('DLV.txt', r)
    IMU = open('IMU.txt', r)
    
    pos = [0, 0, 0]
    vel = [0, 0, 0]
    accel = [0, 0, 0]
    
    
    def __init__(self):
        
        
        
        return
    
    
    
    def predict(self):
        
        velk = self.pos + getSpeed()*self.deltaT + (1/2)*getAccel()*(self.deltaT**2)
        
        
    
    # get reading from DVL
    # provisionally, read from file DVL.txt
    def getSpeed(self):
        
        
    # get reading from IMU
    # provisionally, read from file IMU.txt
    def getAccel(self):
        