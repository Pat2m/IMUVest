import math


class Quaternion:

    def __init__(self, w=1, x=0, y=0, z=0):
        self.set(w, x, y, z)

    def set(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        self.normalize()

    def normalize(self):
        magnitude = self.get_magnitude()
        if magnitude == 0:
            return
        self.w /= magnitude
        self.x /= magnitude
        self.y /= magnitude
        self.z /= magnitude

    def reset(self):
        self.set(1, 0, 0, 0)

    def get_quadruple(self):
        return (self.w, self.x, self.y, self.z)

    def get_magnitude(self):
        return math.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)

    def to_euler(self):
        roll = 0
        d = 2 * (self.w * self.y + self.x * self.z)
        if d != 0:
            pitch = math.asin(d)
        else:
            pitch = 0
        if d == 1:
            yaw = -2 * math.atan2(self.x, self.w)
        elif d == -1:
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
    def hamilton_product(q1, q2):
        wl, xl, yl, zl = q1.get_quadruple()
        wr, xr, yr, zr = q2.get_quadruple()
        w = wl * wr - xl * xr - yl * yr - zl * zr
        x = xl * wl + wl * xr + yl * zr - zl * yr
        y = wl * yr - xl * zr + yl * wl + zl * xr
        z = wl * zr + xl * zr - zl * xr + zl * wl
        return Quaternion(w, x, y, z)

    @staticmethod
    def conjugate(q):
        w, x, y, z = q.get_quadruple()
        x *= -1
        y *= -1
        z *= -1
        return Quaternion(w, x, y, z)

    @staticmethod
    def dot_product(q1, q2):
        wl, xl, yl, zl = q1.get_quadruple()
        wr, xr, yr, zr = q2.get_quadruple()
        return wl * wr + xl * xr + yl * yr + zl * zr

    @staticmethod
    def min_distance(q1, q2):
        if Quaternion.dot_product(q1, q2) >= 0:
            return q2
        w, x, y, z = q2.get_quadruple()
        w *= -1
        x *= -1
        y *= -1
        z *= -1
        return Quaternion(w, x, y, z)

    @staticmethod
    def slerp(q1, q2, weight):
        wl, xl, yl, zl = q1.get_quadruple()
        wr, xr, yr, zr = q2.get_quadruple()
        cos_half_theta = Quaternion.dot_product(q1, q2)

        if abs(cos_half_theta) >= 1:
            return q1

        sin_half_theta = math.sqrt(1 - cos_half_theta ** 2)
        if abs(sin_half_theta) < .001:
            w = wl * .5 + wr * .5
            x = xl * .5 + xr * .5
            y = yl * .5 + yr * .5
            z = zl * .5 + zr * .5
            return Quaternion(w, x, y, z)

        half_theta = math.acos(cos_half_theta)
        r1 = math.sin((1 - weight) * half_theta) / sin_half_theta
        r2 = math.sin(weight * half_theta) / sin_half_theta
        w = wl * r1 + wr * r2
        x = xl * r1 + xr * r2
        y = yl * r1 + yr * r2
        z = zl * r1 + zr * r2
        return Quaternion(w, x, y, z)


q1 = Quaternion(0.5992544, -0.4852698, -0.3969716, 0.4978162)
q2 = Quaternion(0.2569908, 0.4205305, 0.8363884, -0.2399255)
print(Quaternion.slerp(q1, q2, .69420).get_quadruple())
