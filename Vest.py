
import math
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from Quaternion import Quaternion


def sensor_handler(address, *args):
    # print(address, args)
    if address == "/Chordata/raw/kc-CH_6-0":
        prev = Quaternion(0,.7071,.7071,0)
        gx, gy, gz, ax, ay, az, mx, my, mz = args
        Q = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz,
                       .5)
        s = "thigh"
    if address == "/Chordata/raw/kc-CH_6-1":
        prev = Quaternion(0,.7071,.7071,0)
        gx, gy, gz, ax, ay, az, mx, my, mz = args
        Q = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz,
                       .5)
        s = "shin"
    if address == "/Chordata/raw/kc-CH_6-2":
        prev = Quaternion(0, .707, 0, .707)
        gx, gy, gz, ax, ay, az, mx, my, mz = args
        Q = sensor_fuse(prev, gx, gy, gz, ax, ay, az, mx, my, mz,
                       .5)
        s = "foot"
    print(s, ":", Q.get_quadruple())


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


def main():
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/raw/*", sensor_handler)
    ip = "127.0.0.1"
    port = 6565
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()


if __name__ == "__main__":
    main()
