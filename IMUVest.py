from Quaternion import Quaternion
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.linalg import norm
import numpy


class Limb:
    def init(self, length):
        self.length = length
        self.start = (0, 0, 0)
        self.end = (0, 0, 0)
        self.orientation = None
        self.factor = None
        self.inverse = None

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        if self.factor != None:
            prev = Quaternion.hamilton_product(self.inverse, self.orientation)
            prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz, .5)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
        else:
            prev = Quaternion()
            for i in range(10):
                prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, my, mz, .5)
            conjugate = Quaternion.conjugate(prev)
            self.factor = Quaternion.hamilton_product(conjugate, 
                                                      self.orientation)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
            self.inverse = Quaternion.conjugate(self.factor)

    def to_string():
        return "LENGTH:" + str(self.length) + "\tSTART:" + str(self.start) + \
               "\tEND" + str(self.end) + "\tQ:" + str(self.orientation) + \
               "\tFACTOR" + str(self.factor) + "\tINVERSE:" + str(self.inverse)

def update_positions():

    def get_end_position(start, q, length):
        i, j, k = q.to_unit_vector()
        x_0 = i * length
        y_0 = j * length
        z_0 = k * length
        x, y, z = position
        x += x_0
        y += y_0
        z += z_0
        return x, y, z

    def get_orthogonal(start, q, length):
        v = q.to_unit_vector()
        x, y, z = v
        v = numpy.array([x, y, z])
        # make some vector not in the same direction as v
        not_v = numpy.array([1, 0, 0])
        if (v == not_v).all():
            not_v = numpy.array([0, 1, 0])

        # make vector perpendicular to v
        n1 = numpy.cross(v, not_v)
        # normalize n1
        if norm(n1) != 0:
            n1 /= norm(n1)
        else:
            print("Divide by zero error!")

        # make unit vector perpendicular to v and n1
        n2 = numpy.cross(v, n1)
        x, y, z = position
        x += n2[0] * length
        y += n2[1] * length
        z += n2[2] * length
        return x, y, z

    lower_torso.start = (0, 0, left_shin.length + left_thigh.length)
    upper_torso.start = get_end_position(lower_torso.start,
                                         upper_torso.orientation,
                                         upper_torso.length)
    tx, ty, tz = get_orthogonal(lower_torso.start, lower_torso.orientation,
                                hip_width)
    d = tz - lower_torso.position[2]
    left_thigh.start = (-tx, -ty, tz - d)
    left_shin.start = get_end_position(left_thigh.start, 
                                       left_thigh.orientation,
                                       left_thigh.length)
    left_foot.start = get_end_position(left_shin.start, 
                                       left_shin.orientation,
                                       left_shin.length)
    right_thigh.start = (tx, ty, tz)
    right_shin.start = get_end_position(right_thigh.start,
                                        right_thigh.orientation,
                                        right_thigh.length)
    right_foot.start = get_end_position(right_shin.start,
                                        right_shin.orientation,
                                        right_shin.length)
    head.start = get_end_position(upper_torso.start, 
                                  upper_torso.orientation,
                                  upper_torso.length)
    tx, ty, tz = get_orthogonal(head.start, 
                                upper_torso.orientation, 
                                shoulder_width)
    d = tz - head.position[2]
    left_bicep.start = -tx, -ty, tz - d
    left_forearm.start = get_end_position(left_bicep.start,
                                          left_bicep.orientation,
                                          left_bicep.length)
    left_hand.start = get_end_position(left_forearm.start,
                                       left_forearm.orientation,
                                       left_forearm.length)
    right_bicep.start = (tx, ty, tz)
    right_forearm.start = get_end_position(right_bicep.start,
                                              right_bicep.orientation,
                                              right_bicep.length)
    right_hand.start = get_end_position(right_forearm.start,
                                           right_forearm.orientation,
                                           right_forearm.length)
    # TODO: Save end positions.

def sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz, weight):
    x = z = 0

    # Polar Acceleration
    roll = math.atan2(az, ay)
    magnitude = math.sqrt(az ** 2 + ay ** 2)
    tilt = math.atan2(ax, magnitude)
    magnitude = math.sqrt(ax ** 2 + magnitude ** 2) * .00390625

    # Acceleration Quaternion
    w = math.cos(roll * .5)
    y = math.sin(roll * .5)
    qri = Quaternion.conjugate(Quaternion(w, x, y, z))
    w = math.cos(tilt * .5)
    y = math.sin(tilt * .5)
    qti = Quaternion.conjugate(Quaternion(w, x, y, z))
    qa = Quaternion.hamilton_product(qti, qri)

    # Polar Magnometer
    roll = math.atan2(mz, my) - math.pi
    magnitude = math.sqrt(mz ** 2 + my ** 2)
    tilt = math.atan2(mx, magnitude)

    # Magnometer Quaternion
    w = math.cos(roll * .5)
    y = math.cos(roll * .5)
    qr = Quaternion(w, x, y, z)
    w = math.cos(tilt * .5)
    y = math.cos(tilt * .5)
    qt = Quaternion(w, x, y, z)
    qm = Quaternion.hamilton_product(qt, qr)

    # Rotate Quaternion
    qm = Quaternion.hamilton_product(qm, qri)
    qm = Quaternion.hamilton_product(qm, qti)

    # Azimuth
    w, x, y = qm.get_quadruple()[:3]
    azimuth = math.atan2(x, y) + math.pi
    if w > 0:
        azimuth += math.pi
    azimuth = math.fmod(azimuth, 2 * math.pi) - math.pi

    # Finalize Magnometer Quaternion
    w = math.cos(azimuth * .5)
    z = math.sin(azimuth * .5)
    x = y = 0
    qm = Quaternion(w, x, y, z)

    # Combine magnometer and acceletation quaternions
    qam = Quaternion.hamilton_product(qm, qa)

    # Gyro Quaternion
    w = math.cos((gx + gy + gz) * .5)
    x = math.sin(gz * .5)
    y = math.sin(gx * .5)
    z = math.sin(gy * .5)
    qg = Quaternion(w, x, y, z)
    delay = prev = Quaternion.hamilton_product(prev, qg)
    prev = Quaternion.slerp(qam, prev, weight)
    return Quaternion.min_distance(delay, prev)


def sensor_handler(address, *args):
    gx, gy, gz, ax, ay, az, mx, my, mz = args
    # Left Bicep
    if (address == "/Chordata/raw/kc-CH_1-0"):
        left_bicep.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_bicep.to_string())
    # Left Forearm
    elif (address == "/Chordata/raw/kc-CH_1-1"):
        left_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_forearm.to_string())
    # Left Hand
    elif (address == "/Chordata/raw/kc-CH_1-2"):
        left_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_hand.to_string())
    # Head
    elif (address == "/Chordata/raw/kc-CH_2-1"):
        head.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(head.to_string())
    # Upper Torso
    elif (address == "/Chordata/raw/kc-CH_2-2"):
        upper_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(upper_torso.to_string())
    # Right Bicep
    elif (address == "/Chordata/raw/kc-CH_3-0"):
        right_bicep.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_bicep.to_string())
    # Right Forearm
    elif (address == "/Chordata/raw/kc-CH_3-1"):
        right_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_forearm.to_string())
    # Right Hand
    elif (address == "/Chordata/raw/kc-CH_3-2"):
        right_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_hand.to_string())
    # Left Thigh
    elif (address == "/Chordata/raw/kc-CH_4-0"):
        left_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_thigh.to_string())
    # Left Shin
    elif (address == "/Chordata/raw/kc-CH_4-1"):
        left_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_shin.to_string())
    # Left Foot 
    elif (address == "/Chordata/raw/kc-CH_4-2"):
        left_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(left_foot.to_string())
    # Lower Torso
    elif (address == "/Chordata/raw/kc-CH_5-0"):
        lower_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(lower_torso.to_string())
    # Right Thigh
    elif (address == "/Chordata/raw/kc-CH_6-0"):
        right_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_thigh.to_string())
    # Right Shin
    elif (address == "/Chordata/raw/kc-CH_6-1"):
        right_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_shin.to_string())
    # Right Foot
    elif (address == "/Chordata/raw/kc-CH_6-2"):
        right_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        print(right_foot.to_string())
    else:
        print("Unknown address:", address)
    update_positions()


def main():
    # Limbs need to be accessed by sensor handler.
    global left_hand, left_forearm, left_bicep
    global right_hand, right_forearm, right_bicep
    global left_foot, left_shin, left_thigh
    global right_foot, right_shin, right_thigh
    global upper_torso, lower_torso, head

    # Set user length. Initialize Limbs.
    d = input("Hand length: ")
    left_hand = Limb(d)
    right_hand = Limb(d)
    d = input("Forearm length: ")
    left_forearm = Limb(d) 
    right_forearm = Limb(d)
    d = input("Bicep length: ")
    left_bicep = Limb(d)
    right_bicep = Limb(d)
    d = input("Torso length: ")
    upper_torso = Limb(d / 2)
    lower_torso = Limb(d / 2)
    d = input("Thigh length: ")
    left_thigh = Limb(d)
    right_thigh = Limb(d)
    d = input("Shin length: ")
    left_shin = Limb(d)
    right_shin = Limb(d)
    d = input("Foot length: ")
    left_foot = Limb(d)
    right_foot = Limb(d)
    d = input("Head length: ")
    head = Limb(d)

    # Set user width.
    shoulder_width = input("Shoulder width: ")
    hip_width = input("Hip width: ")

    # Calibrate
    input("Stand in a T-pose. Press enter to calibrate.")

    # Set initial orientation.
    left_hand.orientation = Quaternion(0, 0, .7071, .7071)
    left_forearm.orientation = Quaternion(0, 0, .7071, .7071)
    left_bicep.orientation = Quaternion(0, 0, .7071, .7071)
    left_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    left_shin.orientation = Quaternion(0, .7071, .7071, 0)
    left_foot.orientation = Quaternion(0, .7071, 0, .7071)
    right_hand.orientation = Quaternion(0, 0, .7071, -.7071)
    right_forearm.orientation = Quaternion(0, 0, .7071, -.7071)
    right_bicep.orientation = Quaternion(0, 0, .7071, -.7071)
    right_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    right_shin.orientation = Quaternion(0, .7071, .7071, 0)
    right_foot.orientation = Quaternion(0, .7071, 0, .7071)
    head.orientation = Quaternion()
    upper_torso.orientation = Quaternion()
    lower_torso.orientation = Quaternion()

    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/raw/*", sensor_handler)
    ip = "127.0.0.1"
    port = 6565
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()


if __name__ == "__main__":
    main()
