# -*- coding: utf-8 -*-
"""
Created on Thur Sept 8 17:07:23 2022

@author: Pat

TO-DO
Update IMU_VEST to take collider args
    add clock to IMU_VEST for collider objects to update every
    x milliseconds 
Add attach points code to IMU_Vest
    use find endpoint with length/2

"""
from Quaternion import Quaternion
import numpy as np
import math
from scipy.linalg import norm

class Collider():
    def __init__(self, length, distance, stiffness, 
                 dampening, rotational_stiffness,rotational_dampening,
                 orientation = None, start = (0, 0, 0), unit_step=1):
        self.length = length
        self.radius = 1
        self.start = start
        self.distance = distance
        self.orientation = orientation
        self.orthog = self.get_orthogonal(self.start, self.orientation, 1)
        self.stiffness = stiffness
        self.dampening = dampening
        self.rotational_stiffness = rotational_stiffness
        self.rotational_dampening = rotational_dampening
        self.prev_r = None
        self.prev_d = None
        self.r_velocity = (0,0,0,0)
        self.l_velocity = 0
        self.compression = 1
        self.unit_step = unit_step
        self.last_time = None
        

    def q_velocity_mag(self):
        # returned in units of rotation / x milliseconds
        if self.prev_r != None:
            w, x, y, z = self.r_velocity
            return w ** 2 + x ** 2 + y ** 2 + z ** 2
        else: 
            return 0

    def rotatational_oscillation(self, rotation):
        # Check if spring is moving
        dot = Quaternion.dot_product(self.orientation, rotation)
        if dot>= .999 and self.q_velocity_mag() < .00001:
            # Done
            self.orientation = rotation
            return
        # Not Done
        target = Quaternion.min_distance(self.orientation, rotation)

        wO, xO, yO, zO = self.orientation.get_quadruple()
        wT, xT, yT, zT = target.get_quadruple()
        if self.prev_r != None:
            w, x, y, z = self.r_velocity
        else:
            w = x = y = z = 0
        # stiffness * delta q - dampening * velocity 
        w += (-1 * self.rotational_stiffness * (wO - wT) 
               - self.rotational_dampening * w * self.unit_step)
        x += (-1 * self.rotational_stiffness * (xO - xT) 
               - self.rotational_dampening * x * self.unit_step)
        y += (-1 * self.rotational_stiffness * (yO - yT)
               - self.rotational_dampening * y * self.unit_step) 
        z += (-1 * self.rotational_stiffness * (zO - zT)
               - self.rotational_dampening * z * self.unit_step) 

        self.r_velocity = (w, x, y, z)

        wO += w  * self.unit_step
        xO += x  * self.unit_step
        yO += y  * self.unit_step
        zO += z  * self.unit_step

        self.orientation = Quaternion(wO, xO, yO, zO)

    def get_orthogonal(self, start, q, length):
        v = q.to_unit_vector()
        x, y, z = v
        v = np.array([x, y, z])
        # make some vector not in the same direction as v
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])

        # make vector perpendicular to v
        n1 = np.cross(v, not_v)
        # normalize n1
        if norm(n1) != 0:
            n1 /= norm(n1)
        else:
            print("Divide by zero error!")

        # make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        x, y, z = start
        x += n2[0] * length
        y += n2[1] * length
        z += n2[2] * length
        return x, y, z

    def distance_oscillation(self):
        if self.compression >= .999 and self.compression <= 1.001 and self.l_velocity< .00001:
            # Done
            self.compression = 1
            return
        # Not Done
        self.l_velocity = (self.stiffness * (self.distance * self.compression - self.distance) -
                           self.dampening * self.l_velocity * self.unit_step)
        self.compression = 1 / ((self.compression * self.distance) + self.l_velocity
                                * self.unit_step)

    def get_location(self, position, orientation):
        self.start = self.get_orthogonal(position, orientation, self.distance*self.compression)
        x, y, z = self.start
        i, j, k = position
        x -= i
        y -= j
        z -= k
        self.compression = math.sqrt(x**2 + y**2 + z**2) / self.distance 
        self.distance_oscillation()
        self.rotatational_oscillation(orientation)
        self.orthog = self.get_orthogonal(self.start, self.orientation, 1)


       
    def intersect(self, normal_B, pointB, length):
        x, y, z = self.orthog
        normal_A = np.array([x, y, z])
        #print(normal_B)
        x, y, z = normal_B
        normal_B = np.array([x, y, z])

        x, y, z = self.start
        pointA = np.array([x, y, z])

        x, y, z = pointB
        pointB = np.array([x, y, z])

        line_of_intersection = np.cross(normal_A,normal_B)
        n_dot = normal_B.dot(line_of_intersection)
        if abs(n_dot) < .0000001:
            return False
        else:
            for x in range(length):
                for y in range(length):
                    #z1 = (line_of_intersection[0] * x + pointA[0] +
                          #line_of_intersection[1] * y + pointA[1])
                    z1 = (normal_A[0] * x + pointA[0] +
                          normal_A[1] * y + pointA[1])
                    z2 = (normal_B[0] * x + pointB[0] +
                          normal_B[1] * y + pointB[1])
                    if(round(z1,2) == round(z2,2)):
                        return True
            return False



    @staticmethod
    def plane(point, v, mag, R):
        x, y, z = point
        point = np.array([x, y, z])
        v = v.to_unit_vector()
        x, y, z = v
        normal_vector = np.array([x, y, z])
        d = -point.dot(normal_vector)
        x, y = np.meshgrid(range(mag),range(R))
        z = (-normal_vector[0] * x - normal_vector[1] * y - d) * 1.0 / normal_vector[2]