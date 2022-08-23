# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 17:07:23 2022

@author: Pat
"""
import numpy as np
from scipy.linalg import norm
from matplotlib import pyplot as plt
from Quaternion import Quaternion


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

fig = plt.figure()
ax=plt.axes(projection='3d')
ax.axes.set_xlim3d(left=15, right=40) 
ax.axes.set_ylim3d(bottom=15, top=40) 
ax.axes.set_zlim3d(bottom=0, top=30) 
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')


q0 = Quaternion(-0.5504521,-0.0528917,-0.8325403,-0.0328874)
x,y,z = q0.to_unit_vector()
v=np.array([x,y,z])
#axis and radius
p0 = np.array([20, 22, 10]) #point at one end
R = .2
mag = 16
endpoint = p0 + v * mag
x,y,z = drawCir(p0[0],p0[1],p0[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")
s1,t1,b1 = drawCyl(v,p0,R,mag)

x,y,z = drawCir(endpoint[0],endpoint[1],endpoint[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")

ax.plot_surface(s1[0], s1[1], s1[2], color='blue')
ax.plot_surface(t1[0], t1[1], t1[2], color='blue')
ax.plot_surface(b1[0], b1[1], b1[2], color='blue')


p0 = endpoint #point at one end

R = .2
mag = 8
q1 = Quaternion(0.54,0.67,0.41,-0.1387)
qs = Quaternion.slerp(q0,q1,.81)
x,y,z = qs.to_unit_vector()
v1=np.array([x,y,z])

endpoint = p0 + v1 * mag

s1,t1,b1 = drawCyl(v1,p0,R,mag)

x,y,z = drawCir(endpoint[0],endpoint[1],endpoint[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")

ax.plot_surface(s1[0], s1[1], s1[2], color='red')
ax.plot_surface(t1[0], t1[1], t1[2], color='red')
ax.plot_surface(b1[0], b1[1], b1[2], color='red')


x,y,z = q0.to_unit_vector()
v=np.array([x,y,z])
#axis and radius

R = .2
mag = 16
endpoint = p0 + v * mag

s1,t1,b1 = drawCyl(v,p0,R,mag)

x,y,z = drawCir(endpoint[0],endpoint[1],endpoint[2], 150, .5) #Center x,y,z ; surface res; radius
ax.plot_surface(x,y,z,rstride=4, cstride=4, color="yellow")

ax.plot_surface(s1[0], s1[1], s1[2], color='green')
ax.plot_surface(t1[0], t1[1], t1[2], color='green')
ax.plot_surface(b1[0], b1[1], b1[2], color='green')

plt.show(block=True)

