# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 19:15:36 2022

@author: Pat
"""
from Quaternion import Quaternion


class Skeleton:
    _limb_attibutes = ['arm', 'leg', 'torso']
    _side_attibutes = ['left', 'right']
    gravity = 9.81

    def __init__(self, height=3):
        self.height = height
        self.generate_skeleton()
        self.velocity = 0

    def generate_skeleton(self):
        self.left_arm = Limb('arm', 'left')
        self.right_arm = Limb('arm', 'right')
        self.left_leg = Limb('leg', 'left')
        self.right_leg = Limb('leg', 'right')
        self.lower_torso = Limb('torso', 'lower')
        self.upper_torso = Limb('torso', 'upper')
        self.limbs = [self.left_arm, self.right_arm, self.left_leg,
                      self.right_leg, self.lower_torso, self.upper_torso]

    def update_skeleton(self):
        for limb in self.limbs:
            limb.update_limb()


class Limb():
    _arm_attributes = ['hand', 'forearm', 'bicep']
    _leg_attributes = ['heel', 'shin', 'thigh']
    _torso_attributes = ['upper', 'lower']

    def __init__(self, attribute, side):
        self.attribute = attribute
        self.side = side
        self.generate_limb

    def generate_limb(self):
        if self.attribute == 'arm':
            self.hand = Part('hand')
            self.forearm = Part('forearm')
            self.bicep = Part('bicep')
            self.limb = [self.hand, self.forearm, self.bicep]
        if self.attribute == 'leg':
            self.heel = Part('heel')
            self.shin = Part('shin')
            self.thigh = Part('thigh')
            self.limb = [self.heel, self.shin, self.thigh]
        if self.attribute == 'torso':
            self.lower = Part('lower')
            self.upper = Part('upper')
            self.limb = [self.lower, self.upper]

    def update_limb(self):
        for limb in self.limbs:
            limb.update_coordinates()


class Part():
    def __init__(self, attribute, length=1, width=None):
        self.quaternion = Quaternion()
        self.attribute = attribute
        self.length = length
        self.base = [0, 0, 0]
        self.end = [0, 0, length]
        if attribute == 'upper' or 'lower':
            self.width = width
            self.jointL = self.base - [0, self.width/2, 0]
            self.joint_R = self.base + [0, self.width/2, 0]

    def get_length(self):
        return self.length

    def update_base(self, coordinate):
        self.base = coordinate

    def update_coordinates(self):
        self.end = (self.base + self.quaternion.to_unit_vector()
                    * self.length)
        if self.attribute == 'upper' or 'lower':
            self.joint_A = self.base - [0, self.width/2, 0]
            self.joint_B = self.base + [0, self.width/2, 0]
