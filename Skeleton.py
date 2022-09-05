from Quaternion import Quaternion
import numpy as np
from scipy.linalg import norm

class Skeleton:

    def __init__(self, forearm_length, bicep_length, torso_length,
                 thigh_length, shin_length, foot_length, hand_length,
                 hip_width, shoulder_width):
        self.calibrate()
        self.forearm_length = forearm_length
        self.bicep_length = bicep_length
        self.torso_length = torso_length / 2
        self.thigh_length = thigh_length
        self.shin_length = shin_length
        self.foot_length = foot_length
        self.hand_length = hand_length
        self.hip_width = hip_width/2
        self.shoulder_width = shoulder_width/2
        self.head_length = 4
        self.lower_torso_pos = (0, 0, shin_length + thigh_length)
        self.pos_update()

    def pos_update(self):
        self.upper_torso_pos = (Skeleton.get_end_position(self.lower_torso_pos, self.upper_torso, self.torso_length))
        

        x, y, z = Skeleton.get_orthogonal(self.lower_torso_pos, self.lower_torso, self.hip_width)
        i, j, k = self.lower_torso_pos
        dif = z - k
        self.left_thigh_pos = (-x, -y, z - dif)
        self.left_shin_pos = (Skeleton.get_end_position(self.left_thigh_pos, self.left_thigh, self.shin_length))
        self.left_foot_pos = (Skeleton.get_end_position(self.left_shin_pos, self.left_shin, self.shin_length))
        
        self.right_thigh_pos = (x, y, z)
        self.right_shin_pos = (Skeleton.get_end_position(self.right_thigh_pos, self.right_thigh, self.shin_length))
        self.right_foot_pos = (Skeleton.get_end_position(self.right_shin_pos, self.right_shin, self.shin_length))


        self.head_pos = (Skeleton.get_end_position(self.upper_torso_pos, 
                               self.upper_torso, self.torso_length))


        i, j, k = self.head_pos
        x, y, z = Skeleton.get_orthogonal(self.head_pos, self.upper_torso, self.shoulder_width)
        dif = z - k
        self.left_bicep_pos = (-x, -y, z - dif)
        self.left_forearm_pos = (Skeleton.get_end_position(self.left_bicep_pos , self.left_bicep, self.bicep_length))
        self.left_hand_pos = (Skeleton.get_end_position(self.left_forearm_pos , self.left_forearm, self.forearm_length))
        
        self.right_bicep_pos = (x, y, z)
        self.right_forearm_pos = (Skeleton.get_end_position(self.right_bicep_pos , self.right_bicep, self.bicep_length))
        self.right_hand_pos = (Skeleton.get_end_position(self.right_forearm_pos , self.right_forearm, self.forearm_length))
        

    def calibrate(self):
        self.right_hand = Quaternion(0, 0, .707, -.707)
        self.right_forearm = Quaternion(0, 0, .707, -.707)
        self.right_bicep = Quaternion(0, 0, .707, -.707)

        self.left_hand = Quaternion(0, 0, .707, .707)
        self.left_forearm = Quaternion(0, 0, .707, .707)
        self.left_bicep = Quaternion(0, 0, .707, .707)

        self.left_foot = Quaternion(0, .707, 0, .707)
        self.left_shin = Quaternion(0,.7071,.7071,0)
        self.left_thigh = Quaternion(0,.7071,.7071,0)

        self.right_foot = Quaternion(0, .707, 0, .707)
        self.right_shin = Quaternion(0,.7071,.7071,0)
        self.right_thigh = Quaternion(0,.7071,.7071,0)

        self.head = Quaternion()
        self.upper_torso = Quaternion()
        self.lower_torso = Quaternion()
        self.get_inverse()


    def get_skeleton_pos(self):
        return [self.left_hand_pos, self.left_forearm_pos, self.left_bicep_pos,
                self.right_hand_pos, self.right_forearm_pos, self.right_bicep_pos,
                self.left_foot_pos, self.left_shin_pos, self.left_thigh_pos, self.right_foot_pos,
                self.right_shin_pos, self.right_thigh_pos, self.head_pos, self.upper_torso_pos,
                self.lower_torso_pos]
    
    def get_skeleton_length(self):
        return [self.hand_length, self.forearm_length, self.bicep_length,
                self.hand_length, self.forearm_length, self.bicep_length,
                self.foot_length, self.shin_length, self.thigh_length, self.foot_length,
                self.shin_length, self.thigh_length, self.head_length, self.torso_length,
                self.torso_length]

    def get_skeleton_orientation(self):
        return [self.left_hand, self.left_forearm, self.left_bicep,
                self.right_hand, self.right_forearm, self.right_bicep,
                self.left_foot, self.left_shin, self.left_thigh, self.right_foot,
                self.right_shin, self.right_thigh, self.head, self.upper_torso,
                self.lower_torso]
    
    def get_inverse(self):
        self.right_hand_inv = Quaternion.conjugate(self.right_hand)
        self.right_forearm_inv = Quaternion.conjugate(self.right_forearm)
        self.right_bicep_inv = Quaternion.conjugate(self.right_bicep)

        self.left_hand_inv = Quaternion.conjugate(self.left_hand)
        self.left_forearm_inv = Quaternion.conjugate(self.left_forearm)
        self.left_bicep_inv = Quaternion.conjugate(self.left_bicep)

        self.left_foot_inv = Quaternion.conjugate(self.left_foot)
        self.left_shin_inv = Quaternion.conjugate(self.left_shin)
        self.left_thigh_inv = Quaternion.conjugate(self.left_thigh)

        self.right_foot_inv = Quaternion.conjugate(self.right_foot)
        self.right_shin_inv = Quaternion.conjugate(self.right_shin)
        self.right_thigh_inv = Quaternion.conjugate(self.right_thigh)

        self.head_inv = Quaternion.conjugate(self.head)
        self.upper_torso_inv = Quaternion.conjugate(self.upper_torso)
        self.lower_torso_inv = Quaternion.conjugate(self.lower_torso)

    def update(self, **kwargs):
        for key in kwargs.keys:
            
            if key == "left_hand":
                self.left_hand = Quaternion.hamilton_product(kwargs[key], self.left_hand_inv)
            if key == "left_forearm":
                self.left_forearm = Quaternion.hamilton_product(kwargs[key], self.left_forearm_inv)
            if key == "left_bicep":
                self.left_bicep = Quaternion.hamilton_product(kwargs[key], self.left_bicep_inv)

            if key == "right_hand":
                self.right_hand = Quaternion.hamilton_product(kwargs[key], self.right_hand_inv)
            if key == "right_forearm":
                self.right_forearm = Quaternion.hamilton_product(kwargs[key], self.right_forearm_inv)
            if key == "right_bicep":
                self.right_bicep = Quaternion.hamilton_product(kwargs[key], self.right_bicep_inv)

            if key == "left_foot":
                self.left_foot = Quaternion.hamilton_product(kwargs[key], self.left_foot_inv)
            if key == "left_shin":
                self.left_shin = Quaternion.hamilton_product(kwargs[key], self.left_shin_inv)
            if key == "left_thigh":
                self.left_thigh = Quaternion.hamilton_product(kwargs[key], self.left_thigh_inv)

            if key == "right_foot":
                self.right_foot = Quaternion.hamilton_product(kwargs[key], self.right_foot_inv)
            if key == "right_shin":
                self.right_shin = Quaternion.hamilton_product(kwargs[key], self.right_shin_inv)
            if key == "right_thigh":
                self.right_thigh = Quaternion.hamilton_product(kwargs[key], self.right_thigh_inv)

            if key == "head":
                self.head = Quaternion.hamilton_product(kwargs[key], self.head_inv)
            if key == "upper_torso":
                self.upper_torso = Quaternion.hamilton_product(kwargs[key], self.upper_torso_inv)
            if key == "lower_torso":
                self.lower_torso = Quaternion.hamilton_product(kwargs[key], self.lower_torso_inv)
    '''
    def skeleton_update(self, right_hand, right_forearm, right_bicep, left_hand,
                        left_forearm, left_bicep, upper_torso, lower_torso, right_foot,
                        right_shin, right_thigh, left_foot, left_shin, left_thigh):
        self.right_hand = Quaternion.hamilton_product(right_hand, self.right_hand_inv)
        self.right_forearm = Quaternion.hamilton_product(right_forearm, self.right_forearm_inv)
        self.right_bicep = Quaternion.hamilton_product(right_bicep, self.right_bicep_inv)

        self.left_hand = Quaternion.hamilton_product(left_hand, self.left_hand_inv)
        self.left_forearm = Quaternion.hamilton_product(left_forearm, self.left_forearm_inv)
        self.left_bicep = Quaternion.hamilton_product(left_bicep, self.left_bicep_inv)

        self.left_foot = Quaternion.hamilton_product(left_foot, self.left_foot_inv)
        self.left_shin = Quaternion.hamilton_product(left_shin, self.left_shin_inv)
        self.left_thigh = Quaternion.hamilton_product(left_thigh, self.left_thigh_inv)

        self.right_foot = Quaternion.hamilton_product(right_foot, self.right_foot_inv)
        self.right_shin = Quaternion.hamilton_product(right_shin, self.right_shin_inv)
        self.right_thigh = Quaternion.hamilton_product(right_thigh, self.right_thigh_inv)

        self.head = Quaternion.hamilton_product(head, self.head_inv)
        self.upper_torso = Quaternion.hamilton_product(upper_torso, self.upper_torso_inv)
        self.lower_torso = Quaternion.hamilton_product(lower_torso, self.lower_torso_inv)
    '''

    @staticmethod
    def get_end_position(start, q, length):
        i, j, k = q.to_unit_vector()
        x_0 = i * length
        y_0 = j * length
        z_0 = k * length
        x, y, z = start
        x += x_0
        y += y_0
        z += z_0
        return x, y, z

    @staticmethod
    def get_orthogonal(start, q, length):
        v = q.to_unit_vector()
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
        else:
            print("exception")
        
        #make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        x, y, z = start
        x += n2[0] * length
        y += n2[1] * length
        z += n2[2] * length
        return x, y, z