# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 20:11:07 2022

@author: Pat
"""

from matplotlib import pyplot as plt
import numpy as np
from quaternion.QuaternionMath import QuaternionMath
from plotting.plotting import plotting as plotter
from quaternion.Quaternion import Quaternion

qO = Quaternion()
Q = QuaternionMath()
qO.qUpdate(.3779,.333,.5879,-.6329)
#qO.qUpdate(1,0,0,0)
v = Q.qToE(qO)
print(v)
v = plotter.uVector(v)

#axis and radius
p0 = np.array([5, 5, 5]) #point at one end
R = .2
mag = 3


s1,t1,b1 = plotter.drawCyl(v,p0,R,mag)

qO.qUpdate(0.9603,0.1387,0.1981,0.1387)
p0 = np.array([5, 5, 4]) #point at one end
mag = 2
v = Q.qToE(qO)
print(v)
v = plotter.uVector(v)
s2,t2,b2 = plotter.drawCyl(v,p0,R,mag)

ax=plt.subplot(111, projection='3d')

ax.axes.set_xlim3d(left=0, right=10) 
ax.axes.set_ylim3d(bottom=0, top=10) 
ax.axes.set_zlim3d(bottom=0, top=10) 
x,y,z = plotter.drawCir()
ax.plot_surface(x,y,z,color="yellow")
ax.plot_surface(s1[0], s1[1], s1[2], color='blue')
ax.plot_surface(t1[0], t1[1], t1[2], color='blue')
ax.plot_surface(b1[0], b1[1], b1[2], color='blue')

ax.plot_surface(s2[0], s2[1], s2[2], color='red')
ax.plot_surface(t2[0], t2[1], t2[2], color='red')
ax.plot_surface(b2[0], b2[1], b2[2], color='red')

plt.show()