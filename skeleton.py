import math


class Skeleton:

    def __init__(self, left_hand, left_forearm, left_bicep, left_thigh,
                 left_shin, left_foot, right_hand, right_forearm, right_bicep,
                 right_thigh, right_shin, right_foot, upper_torso,
                 lower_torso):
        self.left_hand = left_hand
        self.left_forearm = left_forearm
        self.left_bicep = left_bicep
        self.left_thigh = left_thigh
        self.left_shin = left_shin
        self.left_foot = left_foot
        self.right_hand = right_hand
        self.right_forearm = right_forearm
        self.right_bicep = right_bicep
        self.right_thigh = right_thigh
        self.right_shin = right_shin
        self.right_foot = right_foot
        self.upper_torso = upper_torso
        self.lower_torso = lower_torso


class Limb:

    def __init__(self, magnitude, quaternion):
        self.magnitude = magnitude
        self.quaternion = quaternion


class Quaternion:

    def __init__(self, w=1, x=0, y=0, z=0):
        self.set(w, x, y, z)

    def set(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def get(self):
        return [self.w, self.x, self.y, self.z]

    def reset(self):
        self.update(1, 0, 0, 0)

    def to_euler(self):
        roll = 0
        d = 2 * (self.w * self.y + self.x * self.z)
        if (d != 0):
            pitch = math.asin(d)
        else:
            pitch = 0
        if (d == 1):
            yaw = -2 * math.atan2(self.x, self.w)
        elif (d == -1):
            yaw = 2 * math.atan2(self.x, self.w)
        else:
            yaw = (-1 * math.atan2(2 * (self.w * self.z + self.x * self.y),
                                   1 - 2 * (self.x ** 2 + self.y ** 2))
                   - math.pi / 2)
            roll = (-1 * math.atan2(2 * (self.w * self.x + self.y * self.z),
                                    1 - 2 * (self.y ** 2 + self.z ** 2))
                    - math.pi / 2)
        return round(roll, 4), round(pitch, 4), round(yaw, 4)

    def to_unit_vector(self):
        roll, pitch, yaw = self.to_euler()
        i = (-1 * math.sin(roll) * math.cos(yaw)
             - math.cos(roll) * math.sin(pitch) * math.sin(yaw))
        j = (math.sin(roll) * math.sin(yaw)
             - math.cos(roll) * math.sin(pitch) * math.cos(yaw))
        k = math.cos(roll) * math.cos(pitch)
        return round(i, 4), round(j, 4), round(k, 4)


Q1 = Quaternion(1, 0, 0, 0)
Q2 = Quaternion(.5, -.5, .5, .5)
Q3 = Quaternion(0.3535534, 0.3535534, 0.1464466, 0.8535534)
print(Q1.to_euler(), Q1.to_unit_vector(), "\n",
      Q2.to_euler(), Q2.to_unit_vector(), "\n",
      Q3.to_euler(), Q3.to_unit_vector())
