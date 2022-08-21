# -*- coding: utf-8 -*-
"""
Quaternion Sensor Fusion
Created on Mon Aug 15 14:30:24 2022

@author: Pat
"""

from Quaternion import Quaternion
import numpy as np
import math

class QuaternionMath:
    
    def __init__(self):
        pass
        
    def qMult(self,qL,qR):
        qO = Quaternion()
        ql = qL.qGet()
        qr = qR.qGet()
        w= ql[0]*qr[0] - ql[1]*qr[1] - ql[2]*qr[2] - ql[3]*qr[3]
        x= ql[1]*qr[0] + ql[0]*qr[1] + ql[2]*qr[3] - ql[3]*qr[2]
        y= ql[0]*qr[2] - ql[1]*qr[3] + ql[2]*qr[0] + ql[3]*qr[1]
        z= ql[0]*qr[3] + ql[1]*qr[3] - ql[3]*qr[1] + ql[3]*qr[0]
        qO.qUpdate(w,x,y,z)
        return qO
       
    def qSq(self,q):
        q = q.qGet()
        return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
    
    def qNormalize(self,q):
        qNorm = Quaternion()
        nSq = self.qSq(q)
        q = q.qGet()
        if (nSq == 0):
            nSq = 0.000000001
        w = q[0]/nSq
        x = q[1]/nSq 
        y = q[2]/nSq 
        z = q[3]/nSq 
        qNorm.qUpdate(w,x,y,z) 
        return qNorm
    
    def qInverse(self,q):
        qInv = self.qNormalize(q)
        q = qInv.qGet()
        x = q[1] * -1
        y = q[2] * -1
        z = q[3] * -1
        qInv.qUpdate(q[0],x,y,z) 
        return qInv
    
    def qConjugate(self,q):
        q = q.qGet()
        qConj = Quaternion()
        qConj.qUpdate(q[0], q[1]*-1, q[2]*-1, q[3]*-1)
        return qConj
    
    def qDot(self,qL,qR):
        ql = qL.qGet()
        qr = qR.qGet()
        return ql[0]*qr[0] + ql[1]*qr[1] + ql[2]*qr[2] + ql[3]*qr[3]
    
    def qSlerp(self,qL,qR,weight):
        qTemp = Quaternion()
        halfPi = math.pi/2
        pi = math.pi
        ql = qL.qGet()
        qr = qR.qGet()
        dot = self.qDot(qL, qR)
        if (dot>.995):
            w = ql[0] + (qr[0]-ql[0]) * weight
            x = ql[1] + (qr[1]-ql[1]) * weight
            y = ql[2] + (qr[2]-ql[2]) * weight
            z = ql[3] + (qr[3]-ql[3]) * weight
            qTemp.qUpdate(w,x,y,z)
            norm = self.qNormalize(qTemp)
            return norm
        
        if (dot< -1):
            dot = -1
            
        theta0 = math.acos(dot)
        if (theta0 > 0 and theta0 < halfPi):
            theta = theta0 * weight
        else:
            theta = (theta0-pi)*weight
        
        w = qr[0]-ql[0] * dot
        x = qr[1]-ql[1] * dot
        y = qr[2]-ql[2] * dot
        z = qr[3]-ql[3] * dot
        
        qTemp.qUpdate(w,x,y,z)
        norm = self.qNormalize(qTemp) 
        
        n = norm.qGet()
        cos =math.cos(theta)
        sin =math.sin(theta)
        w = ql[0] * cos+ n[0] * sin
        x = ql[1] * cos+ n[1] * sin
        y = ql[2] * cos+ n[2] * sin
        z = ql[3] * cos+ n[3] * sin
        
        qTemp.qUpdate(w,x,y,z)
        
        return qTemp
    
    def qCopy(self,q):
        q = q.qGet()
        newQ = Quaternion()
        newQ.qUpdate(q[0], q[1], q[2], q[3])
        return newQ
        
    def qMin(self,qL,qR):
        if (self.qDot(qL,qR)<0):
            q = qR.qGet()
            w = q[0] * -1
            x = q[1] * -1
            y = q[2] * -1
            z = q[3] * -1
            qR.qUpdate(w,x,y,z) 
            return qR
        
    def qToE(self,q):
        q = q.qGet()
        r = 0
        halfPi = math.pi/2
        p = math.asin((2*(q[0]*q[2]+q[1]*q[3])))
        if (p == halfPi):
            y = -2 * math.atan2(q[1],q[0])
        elif (p == -1* halfPi):
            y = 2 * math.atan2(q[1],q[0])
        else:
            y = -1* math.atan2((2*(q[0]*q[3]+q[1]*q[2])), 1-2*(q[1]**2 + q[2]**2)) - halfPi # (q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)
            r = -1* math.atan2((2*(q[0]*q[1]+q[2]*q[3])), 1-2*(q[3]**2 + q[2]**2))  #(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
        
        euler = np.array([r,p,y])
        return euler







