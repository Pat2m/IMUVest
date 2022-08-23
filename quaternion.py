import math


class Quaternion:

    def __init__(self, w=1, x=0, y=0, z=0):
        self.set(w, x, y, z)

    def set(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def to_quadruple(self):
        return (self.w, self.x, self.y, self.z)

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
        return (round(roll, 4), round(pitch, 4), round(yaw, 4))

    def to_unit_vector(self):
        roll, pitch, yaw = self.to_euler()
        i = (-1 * math.sin(roll) * math.cos(yaw)
             - math.cos(roll) * math.sin(pitch) * math.sin(yaw))
        j = (math.sin(roll) * math.sin(yaw)
             - math.cos(roll) * math.sin(pitch) * math.cos(yaw))
        k = math.cos(roll) * math.cos(pitch)
        return (round(i, 4), round(j, 4), round(k, 4))

    @staticmethod
    def multiply(q1, q2):
        q1 = q1.to_quadruple()
        q2 = q2.to_quadruple()
        w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
        x = q1[1]*q2[0] + q1[0]*q2[1] + q1[2]*q2[3] - q1[3]*q2[2]
        y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
        z = q1[0]*q2[3] + q1[1]*q2[3] - q1[3]*q2[1] + q1[3]*q2[0]
        return Quaternion(w, x, y, z)
