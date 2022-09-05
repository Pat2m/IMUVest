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
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x ** 2 + self.y ** 2)
        roll_x = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (self.w * self.y - self.z * self.x)
        if (abs(sinp) >= 1):
            pitch_y = math.copysign(math.pi / 2, sinp)
        else:
            pitch_y = math.asin(sinp)

        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y ** 2 + self.z ** 2)
        yaw_z = math.atan2(siny_cosp, cosy_cosp)
        return roll_x, pitch_y, yaw_z

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
        rl = math.sin((1 - weight) * half_theta) / sin_half_theta
        rr = math.sin(weight * half_theta) / sin_half_theta
        w = wl * rl + wr * rr
        x = xl * rl + xr * rr
        y = yl * rl + yr * rr
        z = zl * rl + zr * rr
        return Quaternion(w, x, y, z)
