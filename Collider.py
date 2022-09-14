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

class Collider():
    def __init__(self, length = 1, radius = 1, distance, start = (0,0,0), orientation = None,
                 stiffness, dampening, rotational_stiffness,rotational_dampening,
                 unit_step=1):
        self.length = length
        self.radius = radius
        self.start = start
        self.distance = distance
        self.orientation = orientation
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
        

    def q_velocity_mag():
        # returned in units of rotation / x milliseconds
        if self.prev_r != None:
            w, x, y, z = self.r_velocity
            return w ** 2 + x ** 2 + y ** 2 + z ** 2
        else: 
            return 0

    def rotatational_oscillation(rotation):
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
            w, x, y, z = 0
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

    def distance_oscillation()
        if self.compression >= .999 and self.compression <= 1.001 and self.l_velocity< .00001:
            # Done
            self.compression = 1
            return
        # Not Done
        self.l_velocity = (self.stiffness * (1/self.compression - self.distance) -
                           self.dampening * self.l_velocity * self.unit_step)
        self.compression = 1 / ((self.compression * self.distance) + self.l_velocity
                                * self.unit_step)

    def get_location(position, orientation):
        self.distance_oscillation()
        self.rotatational_oscillation(orientation)
        self.start = Skeleton.get_orthogonal(position, orientation, self.distance*self.compression)


    # returns a boolean whether 2 planes are intersecting
    @staticmethod          
    def intersect(normal_A, pointA, normal_B, pointB, length):
        line_of_intersection = np.cross(normal_A,normal_B)
        n_dot = normal_B.dot(line_of_intersection)
        if abs(n_dot) < .0000001:
            return False
        else:
            for x in range(length):
                for y in range(length):
                    z1 = (line_of_intersection[0] * x + pointA[0] +
                          line_of_intersection[1] * y + pointA[1])
                    z2 = (normal_B[0] * x + pointB[0] +
                          normal_B[1] * y + pointB[1])
                    if(z1 == z2):
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