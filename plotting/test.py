# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 16:26:11 2022

@author: Pat
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Oct  2 18:33:10 2016

Modified from https://stackoverflow.com/questions/38076682/how-to-add-colors-to-each-individual-face-of-a-cylinder-using-matplotlib
to add "end caps" and to undo fancy coloring.

@author: astrokeat
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy.linalg import norm
from QuaternionMath import QuaternionMath
import math as m
from Quaternion import Quaternion

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



def postition(start, vector, mag):
    xdata = np.linspace(start[0],start[0] + mag*vector[0],100)
    ydata = np.linspace(start[1],start[1] + mag*vector[0],100)
    zdata = np.linspace(start[2],start[2] + mag*vector[0],100)
    return xdata, ydata, zdata


qO = Quaternion()
Q = QuaternionMath()
qO.qUpdate(.3779,.333,.5879,-.6329)
#qO.qUpdate(1,0,0,0)
#qO.qUpdate(0.9603,0.1387,0.1981,0.1387)
v = Q.qToE(qO)
print(v)
v = uVector(v)

#axis and radius
p0 = np.array([1, 3, 2]) #point at one end
p1 = np.array([8, 2,1]) #point at other end
R = .2

# #vector in direction of axis
# v = p1 - p0

# #find magnitude of vector
# mag = norm(v)
mag = 3
# #unit vector in direction of axis
# v = v / mag

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

ax=plt.subplot(111, projection='3d')

ax.axes.set_xlim3d(left=0, right=10) 
ax.axes.set_ylim3d(bottom=0, top=10) 
ax.axes.set_zlim3d(bottom=0, top=10) 

ax.plot_surface(X, Y, Z, color='blue')
ax.plot_surface(X2, Y2, Z2, color='blue')
ax.plot_surface(X3, Y3, Z3, color='blue')

plt.show()

