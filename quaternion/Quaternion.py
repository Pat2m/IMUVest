# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 14:51:44 2022

@author: Pat
"""
import numpy as np

class Quaternion:
    
    def __init__(self):
        self.w = 1
        self.x = 0
        self.y = 0
        self.z = 0
        
    def qUpdate(self,w,x,y,z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        
    def qGet(self):
        q = np.array([self.w,self.x,self.y,self.z])
        return q
        
    def qClear(self):
        self.w = 1
        self.x = 0
        self.y = 0
        self.z = 0