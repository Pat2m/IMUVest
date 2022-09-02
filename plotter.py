# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 17:07:23 2022

@author: Pat
"""
import numpy as np
from scipy.linalg import norm
from matplotlib import pyplot as plt
from Quaternion import Quaternion
from Skeleton import Skeleton

def drawCir(point,n,scale):
    x, y, z = point
    u = np.linspace(0, 2*np.pi, n)
    v = np.linspace(0, np.pi, n)
    x =  scale * np.outer(np.cos(u), np.sin(v)) + x
    y =  scale * np.outer(np.sin(u), np.sin(v)) + y
    z =  scale * np.outer(np.ones(np.size(u)), np.cos(v)) + z
    return x,y,z

def drawCyl(p0, v, mag, R):
    v = v.to_unit_vector()
    x, y, z = v
    v = np.array([x, y, z])
    #make some vector not in the same direction as v
    not_v = np.array([1, 0, 0])
    if (v == not_v).all():
            not_v = np.array([0, 1, 0])
    
    #make vector perpendicular to v
    n1 = np.cross(v, not_v)
    #normalize n1
    if norm(n1) != 0:
            n1 /= norm(n1)
            #make unit vector perpendicular to v and n1
            n2 = np.cross(v, n1)
    else:
        n1 = [0,1,0]
        n2 = np.cross(v, n1)
        print("exception")
    
    #surface ranges over t from 0 to length of axis and 0 to 2*pi
    t = np.linspace(0, mag, 2)
    theta = np.linspace(0, 2 * np.pi, 100)
    rsample = np.linspace(0, R, 2)
    
    #use meshgrid to make 2d arrays
    t, theta2 = np.meshgrid(t, theta)
    
    rsample,theta = np.meshgrid(rsample, theta)
    x, y, z = p0
    p0 = np.array([x, y, z]) 
    print(n1)
    print(n2)
    #generate coordinates for surface
    # "Tube"
    X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta2) * n1[i] + R * np.cos(theta2) *       n2[i] for i in [0, 1, 2]]
    # "Bottom"
    X2, Y2, Z2 = [p0[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    # "Top"
    X3, Y3, Z3 = [p0[i] + v[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    return [X,Y,Z], [X2,Y2,Z2], [X3,Y3,Z3]

def plot_skeleton(ax, Skeleton):
    pos = Skeleton.get_skeleton_pos()
    v = Skeleton.get_skeleton_orientation()
    l = Skeleton.get_skeleton_length()
    for i in range(len(pos)):
        x, y, z = drawCir(pos[i], 100, 1)
        ax.plot_surface(x, y, z,rstride=4, cstride=4, color="yellow")

        s1,t1,b1 = drawCyl(pos[i], v[i], l[i], .5)
        ax.plot_surface(s1[0], s1[1], s1[2], color='green')
        ax.plot_surface(t1[0], t1[1], t1[2], color='green')
        ax.plot_surface(b1[0], b1[1], b1[2], color='green')
        
    x, y, z = drawCir(Skeleton.get_end_position(Skeleton.head_pos, Skeleton.head, Skeleton.head_length),
                       100, 2)
    ax.plot_surface(x, y, z,rstride=4, cstride=4, color="yellow")
    return ax

fig = plt.figure()
ax=plt.axes(projection='3d')
ax.axes.set_xlim3d(left=-15, right=15) 
ax.axes.set_ylim3d(bottom=-15, top=15) 
ax.axes.set_zlim3d(bottom=0, top=30) 
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

skeleton = Skeleton(4, 4, 10, 10, 10, 3, 2, 5, 8)
ax = plot_skeleton(ax, skeleton)
plt.show(block=True)