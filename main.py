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
import matplotlib.animation as animation

def animate():
    # 360 Degree view
    for angle in range(0, 360):
       ax.view_init(angle, 30)
       plt.draw()
       plt.pause(.001)

fig = plt.figure()
ax=plt.axes(projection='3d')
ax.axes.set_xlim3d(left=0, right=30) 
ax.axes.set_ylim3d(bottom=0, top=30) 
ax.axes.set_zlim3d(bottom=0, top=30) 
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')


qO = Quaternion()
Q = QuaternionMath()
qO.qUpdate(0.5,0.5,0.5,.5)
#qO.qUpdate(0.9603,0.1387,0.1981,0.1387)
v = Q.qToE(qO)
v = plotter.uVector(v)

#axis and radius
p0 = np.array([20, 22, 0]) #point at one end
R = .2
mag = 8
endpoint = p0 + v * mag

s1,t1,b1 = plotter.drawCyl(v,p0,R,mag)

x,y,z = plotter.drawCir(endpoint[0],endpoint[1],endpoint[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")

ax.plot_surface(s1[0], s1[1], s1[2], color='blue')
ax.plot_surface(t1[0], t1[1], t1[2], color='blue')
ax.plot_surface(b1[0], b1[1], b1[2], color='blue')

qO.qUpdate(.5,-.5,.5,.5)
#qO.qUpdate(.3779,.333,.5879,-.6329)
p0 = endpoint #point at one end
mag = 25
v = Q.qToE(qO)
v = plotter.uVector(v)
endpoint = p0 + v * mag

s2,t2,b2 = plotter.drawCyl(v,p0,R,mag)

x,y,z = plotter.drawCir(endpoint[0],endpoint[1],endpoint[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")


ax.plot_surface(s2[0], s2[1], s2[2], color='red')
ax.plot_surface(t2[0], t2[1], t2[2], color='red')
ax.plot_surface(b2[0], b2[1], b2[2], color='red')

#qO.qUpdate(1,0,0,0)
p0 = endpoint #point at one end
mag = 20
qO.qUpdate(.969,-.038,.177,-.166)
v = Q.qToE(qO)
v = plotter.uVector(v)
endpoint = p0 + v * mag

s2,t2,b2 = plotter.drawCyl(v,p0,R,mag)


ax.plot_surface(s2[0], s2[1], s2[2], color='green')
ax.plot_surface(t2[0], t2[1], t2[2], color='green')
ax.plot_surface(b2[0], b2[1], b2[2], color='green')



plt.show()