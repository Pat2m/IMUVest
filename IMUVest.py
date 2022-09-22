from Quaternion import Quaternion
from collider import Collider
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.linalg import norm
import numpy
import socket
import math
import serial
import time

class Limb():
    def __init__(self, name):
        self.length = 1
        self.start = (0, 0, 0)
        self.end = (0, 0, 0)
        self.orientation = None
        self.factor = None
        self.inverse = None
        self.name = name
        self.midpoint = None
        self.orthog = None

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        if self.factor != None:
            prev = Quaternion.hamilton_product(self.inverse, self.orientation)
            prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
        else:
            prev = Quaternion()
            for i in range(10):
                prev = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz)
            conjugate = Quaternion.conjugate(prev)
            self.factor = Quaternion.hamilton_product(conjugate, 
                                                      self.orientation)
            self.orientation = Quaternion.hamilton_product(prev, self.factor)
            self.inverse = Quaternion.conjugate(self.factor)

    def to_string(self):
        return "LENGTH:" + str(self.length) + "\tSTART:" + str(self.start) + \
               "\tEND" + str(self.end) + "\tQ:" + str(self.orientation) + \
               "\tFACTOR" + str(self.factor) + "\tINVERSE:" + str(self.inverse) + "\n"
    
    def to_json(self):
        return  str(
                    { "Limb": self.name,
		      "data": {"orientation": str(self.orientation),
                               "start": str(self.start),
                               "end": str(self.end)
                              }
                }
            )
            


def update_positions():

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

    def get_mid_position(limb):
        x, y, z = limb.start 
        i, j, k = limb.end
        x = (i - x) / 2
        y = (j - y) / 2
        z = (k - z) / 2
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
        x, y, z = start
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
    d = tz - lower_torso.start[2]
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
    d = tz - head.start[2]
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
    # Save end positions.
    lower_torso.end = upper_torso.start
    upper_torso.end = head.start
    right_bicep.end = right_forearm.start
    right_forearm.end = right_hand.start
    right_hand.end = get_end_position(right_hand.start,
                                           right_hand.orientation,
                                           right_hand.length)
    right_thigh.end = right_shin.start
    right_shin.end = right_foot.start 
    right_foot.end = get_end_position(right_foot.start,
                                           right_foot.orientation,
                                           right_foot.length)
    left_bicep.end = left_forearm.start
    left_forearm.end = left_hand.start
    left_hand.end = get_end_position(left_hand.start,
                                           left_hand.orientation,
                                           left_hand.length)
    left_thigh.end = left_shin.start
    left_shin.end = left_foot.start 
    left_foot.end = get_end_position(left_foot.start,
                                           left_foot.orientation,
                                           left_foot.length)

    lower_torso.midpoint = get_mid_position(lower_torso)
    upper_torso.midpoint = get_mid_position(upper_torso)
    right_bicep.midpoint = get_mid_position(right_bicep)
    right_forearm.midpoint = get_mid_position(right_forearm)
    right_hand.midpoint = get_mid_position(right_hand)
    right_thigh.midpoint = get_mid_position(right_thigh)
    right_shin.midpoint = get_mid_position(right_shin)
    right_foot.midpoint = get_mid_position(right_foot)
    left_bicep.midpoint = get_mid_position(left_bicep)
    left_forearm.midpoint = get_mid_position(left_forearm)
    left_hand.midpoint = get_mid_position(left_hand)
    left_thigh.midpoint = get_mid_position(left_thigh)
    left_shin.midpoint = get_mid_position(left_shin)
    left_foot.midpoint = get_mid_position(left_foot)

    right_forearm.orthog = get_orthogonal(right_forearm.midpoint, right_forearm.orientation, 1)

    
def update_colliders():
    def update(limb, collider):
        collider.get_location(limb.midpoint, limb.orientation)
        return collider.intersect(limb.orthog, limb.midpoint, limb.length)

    timer_delay = 60
    result = update(right_forearm, right_forearm_collider)
    print("result\n")
    fileDump.write(str(result) + "\n")
    if ard:
        current = time.time()
        print("in ard\n")
        fileDump.write("in ard\n")
        if result:
            print("in result\n")
            fileDump.write("in result\n")
            if right_forearm_collider.last_time == None:
                right_forearm_collider.last_time = current
                print("event triggered")
                fileDump.write("event triggered\n")
                arduino.write(b'H')
            elif current - right_forearm_collider.last_time > timer_delay:
                right_forearm_collider.last_time = current
                print("event triggered")
                fileDump.write("event triggered\n")
                arduino.write(b'H')
    
    

def sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz, weight = .5):
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
        if bluetooth:
	        s.send(bytes(left_bicep.to_json(), 'UTF-8'))
        print(left_bicep.to_string())
        fileDump.write(left_bicep.to_string())
    # Left Forearm
    elif (address == "/Chordata/raw/kc-CH_1-1"):
        left_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_forearm.to_json(), 'UTF-8'))
        print(left_forearm.to_string())
        fileDump.write(left_forearm.to_string())
    # Left Hand
    elif (address == "/Chordata/raw/kc-CH_1-2"):
        left_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_hand.to_json(), 'UTF-8'))
        print(left_hand.to_string())
        fileDump.write(left_hand.to_string())
    # Head
    elif (address == "/Chordata/raw/kc-CH_2-1"):
        head.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(head.to_json(), 'UTF-8'))
        print(head.to_string())
        fileDump.write(head.to_string())
    # Upper Torso
    elif (address == "/Chordata/raw/kc-CH_2-2"):
        upper_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(upper_torso.to_json(), 'UTF-8'))
        print(upper_torso.to_string())
        fileDump.write(upper_torso.to_string())
    # Right Bicep
    elif (address == "/Chordata/raw/kc-CH_3-0"):
        right_bicep.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_bicep.to_json(), 'UTF-8'))
        print(right_bicep.to_string())
        fileDump.write(right_bicep.to_string())
    # Right Forearm
    elif (address == "/Chordata/raw/kc-CH_3-1"):
        right_forearm.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_forearm.to_json(), 'UTF-8'))
        print(right_forearm.to_string())
        fileDump.write(right_forearm.to_string())
    # Right Hand
    elif (address == "/Chordata/raw/kc-CH_3-2"):
        right_hand.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_hand.to_json(), 'UTF-8'))
        print(right_hand.to_string())
        fileDump.write(right_hand.to_string())
    # Left Thigh
    elif (address == "/Chordata/raw/kc-CH_4-0"):
        left_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_thigh.to_json(), 'UTF-8'))
        print(left_thigh.to_string())
        fileDump.write(left_thigh.to_string())
    # Left Shin
    elif (address == "/Chordata/raw/kc-CH_4-1"):
        left_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_shin.to_json(), 'UTF-8'))
        print(left_shin.to_string())
        fileDump.write(left_shin.to_string())
    # Left Foot 
    elif (address == "/Chordata/raw/kc-CH_4-2"):
        left_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(left_foot.to_json(), 'UTF-8'))
        print(left_foot.to_string())
        fileDump.write(left_foot.to_string())
    # Lower Torso
    elif (address == "/Chordata/raw/kc-CH_5-0"):
        lower_torso.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(lower_torso.to_json(), 'UTF-8'))
        print(lower_torso.to_string())
        fileDump.write(lower_torso.to_string())
    # Right Thigh
    elif (address == "/Chordata/raw/kc-CH_6-0"):
        right_thigh.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_thigh.to_json(), 'UTF-8'))
        print(right_thigh.to_string())
        fileDump.write(right_thigh.to_string())
    # Right Shin
    elif (address == "/Chordata/raw/kc-CH_6-1"):
        right_shin.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_shin.to_json(), 'UTF-8'))
        print(right_shin.to_string())
        fileDump.write(right_shin.to_string())
    # Right Foot
    elif (address == "/Chordata/raw/kc-CH_6-2"):
        right_foot.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        if bluetooth:
            s.send(bytes(right_foot.to_json(), 'UTF-8'))
        print(right_foot.to_string())
        fileDump.write(right_foot.to_string())
    else:
        print("Unknown address:", address)
        fileDump.write("Unknown address:", address)
    update_positions()
    if collide:
        update_colliders()


def main():
    # Limbs need to be accessed by sensor handler.
    global left_hand, left_forearm, left_bicep
    global right_hand, right_forearm, right_bicep
    global left_foot, left_shin, left_thigh
    global right_foot, right_shin, right_thigh
    global upper_torso, lower_torso, head, s
    global shoulder_width, hip_width
    global bluetooth, ard, collide
    global arduino, fileDump

    fileDump = open("algDump.txt", "w+")

    bluetooth = False
    ard = True
    collide = True

    if ard:
        arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=.1)
        print("Arduino Started")
        fileDump.write("Arduino Started\n")
        arduino.write(b'H')

    left_hand = Limb("left_hand")
    left_forearm = Limb("left_forearm")
    left_bicep = Limb("left_bicep")
    right_hand = Limb("right_hand")
    right_forearm = Limb("right_forearm")
    right_bicep = Limb("right_bicep")
    left_foot = Limb("left_foot")
    left_shin = Limb("left_shin")
    left_thigh = Limb("left_thigh")
    right_foot = Limb("right_foot")
    right_shin = Limb("right_shin")
    right_thigh = Limb("right_thigh")
    upper_torso = Limb("upper_torso")
    lower_torso = Limb("lower_torso")
    head = Limb("head")
    # Set user length. Initialize Limbs.
    d = input("Hand length: ")
    left_hand.length = int(d)
    right_hand.length = int(d)
    d = input("Forearm length: ")
    left_forearm.length = int(d) 
    right_forearm.length = int(d)
    d = input("Bicep length: ")
    left_bicep.length = int(d)
    right_bicep.length = int(d)
    d = input("Torso length: ")
    upper_torso.length = int(d) / 2
    lower_torso.length = int(d) / 2
    d = input("Thigh length: ")
    left_thigh.length = int(d)
    right_thigh.length = int(d)
    d = input("Shin length: ")
    left_shin.length = int(d)
    right_shin.length = int(d)
    d = input("Foot length: ")
    left_foot.length = int(d)
    right_foot.length = int(d)
    d = input("Head length: ")
    head.length = int(d)

    # Set user width.
    shoulder_width = int(input("Shoulder width: "))
    hip_width = int(input("Hip width: "))

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

    update_positions()

    if collide:
        global left_hand_collider, left_forearm_collider, left_bicep_collider
        global right_hand_collider, right_forearm_collider, right_bicep_collider
        global left_foot_collider, left_shin_collider, left_thigh_collider
        global right_foot_collider, right_shin_collider, right_thigh_collider
        global upper_torso_collider, lower_torso_collider, head_collider

        global left_hand_collider_enable, left_forearm_collider_enable, left_bicep_collider_enable
        global right_hand_collider_enable, right_forearm_collider_enable, right_bicep_collider_enable
        global left_foot_collider_enable, left_shin_collider_enable, left_thigh_collider_enable
        global right_foot_collider_enable, right_shin_collider_enable, right_thigh_collider_enable
        global upper_torso_collider_enable, lower_torso_collider_enable, head_collider_enable

        left_hand_collider_enable = False
        left_forearm_collider_enable = False
        left_bicep_collider_enable = False
        right_hand_collider_enable = False
        right_forearm_collider_enable = False
        right_bicep_collider_enable = False
        left_foot_collider_enable = False
        left_shin_collider_enable = False
        left_thigh_collider_enable = False
        right_foot_collider_enable = False
        right_shin_collider_enable = False
        right_thigh_collider_enable = False
        upper_torso_collider_enable = False
        lower_torso_collider_enable = False
        head_collider_enable = False

        right_forearm_collider = Collider(right_forearm.length, .25, 
                 1, 0, 1, 0, right_forearm.orientation, right_forearm.midpoint)

    
    if bluetooth:
        #serverMACAddress = 'A4:B1:C1:33:DB:27' # Desktop
        serverMACAddress = '30:24:32:7D:36:46' # Laptop
        port = 4
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.connect((serverMACAddress,port))
    
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/raw/*", sensor_handler)
    ip = "127.0.0.1"
    port = 6565
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()


if __name__ == "__main__":
    main()

