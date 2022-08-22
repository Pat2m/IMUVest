# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 16:26:11 2022

@author: Pat
"""

import numpy as np
from scipy.linalg import norm
import math as m

class plotting():
    def uVector(orientation):
        x = -1*m.sin(orientation[0])*m.cos(orientation[2]) - m.cos(orientation[0])*m.sin(orientation[1])*m.sin(orientation[2])
        y = m.sin(orientation[0])*m.sin(orientation[2]) - m.cos(orientation[0])*m.sin(orientation[1])*m.cos(orientation[2])
        z = m.cos(orientation[0]) * m.cos(orientation[1])
        # r = orientation[0]
        # p = orientation[1]
        # y = orientation[2]
        # k = [m.cos(y)*m.cos(p), m.sin(p), m.sin(y)*m.cos(p)]
        # y = [0, 1, 0]
        # s = np.cross(k,y)
        # v = np.cross(s,k)
        # vrot = v* m.cos(r) + np.cross(k, v)* m.sin(r)
        return [x,y,z]
    
    def drawCir(center_x,center_y,center_z,n,scale):
        u = np.linspace(0, 2*np.pi, n)
        v = np.linspace(0, np.pi, n)
        x =  scale * np.outer(np.cos(u), np.sin(v)) + center_x
        y =  scale * np.outer(np.sin(u), np.sin(v)) + center_y
        z =  scale * np.outer(np.ones(np.size(u)), np.cos(v)) + center_z
        return x,y,z
    
    def drawCyl(v, p0, R, mag):
        #make some vector not in the same direction as v
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
                not_v = np.array([0, 1, 0])
        
        #make vector perpendicular to v
        n1 = np.cross(v, not_v)
        #normalize n1
        n1 /= norm(n1)
        
        #make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        
        #surface ranges over t from 0 to length of axis and 0 to 2*pi
        t = np.linspace(0, mag, 2)
        theta = np.linspace(0, 2 * np.pi, 100)
        rsample = np.linspace(0, R, 2)
        
        #use meshgrid to make 2d arrays
        t, theta2 = np.meshgrid(t, theta)
        
        rsample,theta = np.meshgrid(rsample, theta)
        
        #generate coordinates for surface
        # "Tube"
        X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta2) * n1[i] + R * np.cos(theta2) *       n2[i] for i in [0, 1, 2]]
        # "Bottom"
        X2, Y2, Z2 = [p0[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        # "Top"
        X3, Y3, Z3 = [p0[i] + v[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        return [X,Y,Z], [X2,Y2,Z2], [X3,Y3,Z3] 
    

