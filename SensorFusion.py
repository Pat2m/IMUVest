# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 18:40:18 2022

@author: Pat
"""

import math as m
import Quaternion
import QuaternionMath as Q

class SensorFusion:
    
    def __init__(self):
        pass
    
    def sensorFuse(self, node, ax, ay, az, gx, gy, gz, mx, my, mz, weight):
        pi = m.Pi()
        # Polar Acceleration
        roll = m.atan2(az, ay)
        mag = m.sqrt(az**2 + ay**2)
        tilt = m.atan2(ax, mag)
        mag = m.sqrt(ax**2 + mag**2)
        mag = mag *.00390625
        
        # Acceleration Quaternion
        qAr = qAt = Quaternion()
        w = m.cos(roll*.5)
        y = m.sin(roll*.5)
        x = z = 0
        qAr.qUpdate(w,x,y,z)
        qInvL = Q.qInverse(qAr)
        w = m.cos(tilt*.5)
        y = m.sin(tilt*.5)
        qAt.qUpdate(w,x,y,z)
        qInvR = Q.qInverse(qAt)
        qAc = Q.qMult(qInvR,qInvL)
        
        # Polar Mag
        roll = m.atan2(mz, my) - pi
        mag = m.sqrt(mz**2 + my**2)
        tilt = m.atan2(mx, mag)
        
        # Mag Quaternion
        qMr = qMt = Quaternion()
        w = m.cos(roll*.5)
        y = m.sin(roll*.5)
        x = z = 0
        qMr.qUpdate(w,x,y,z)
        w = m.cos(tilt*.5)
        y = m.sin(tilt*.5)
        qMt.qUpdate(w,x,y,z)
        qM = Q.qMult(qMt, qMr)
        
        # Rotate Quaternion
        qM = Q.qMult(qM, qInvL)
        qM = Q.qMult(qM, qInvR)
        
        # Azimuth Calcs
        pos = qM.qGet()
        azimuth = m.atan2(pos[1], pos[2]) + pi
        if (pos[0] > 0):
            azimuth = azimuth + pi
        azimuth = m.fmod(azimuth, 2*pi) - pi
        
        # Finalize Mag Quaternion
        w = m.cos(azimuth *.5)
        z = m.sin(azimuth *.5)
        x = y = 0
        qM.qUpdate(w,x,y,z)
        
        # Combine qM and qAc
        qAM = Q.qMult(qM, qAc)
        
        # Gyro Quaternion
        qG = Quaternion()
        w = m.cos((gx + gy + gz) * .5)
        x = m.sin(gz * .5)
        y = m.sin(gx * .5)
        z = m.sin(gy * .5)
        qG.qUpdate(w,x,y,z)
        
        
        # Complementary filter
        # combine previous with gyro
        delay = node = Q.qMult(node,qG)
        # slerp acc & mag estimate with previous orientation of node
        node = Q.slerp(qAM, node, weight)
        
        # use the Shortest distance between orientations
        node = Q.qMin(delay, node)
        
        return node
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    