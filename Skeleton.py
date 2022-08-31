from Quaternion import Quaternion


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
        self.hip_width = hip_width
        self.shoulder_width = shoulder_width

        self.left_foot_pos = (0, hip_width / -2, 0)
        self.left_shin_pos = (0, hip_width / -2, shin_length)
        self.left_thigh_pos = (0, hip_width / -2, shin_length + thigh_length)

        self.right_foot_pos = (0, hip_width / 2, 0)
        self.right_shin_pos = (0, hip_width / 2, shin_length)
        self.right_thigh_pos = (0, hip_width / 2, shin_length + thigh_length)

        self.lower_torso_pos = (0, 0, shin_length + thigh_length)
        self.upper_torso_pos = (0, 0, shin_length + thigh_length
                                + torso_length)

        self.left_bicep_pos = (0, shoulder_width / -2, shin_length
                               + thigh_length + torso_length * 2)
        self.left_forearm_pos = (0, shoulder_width / -2 - bicep_length,
                                 shin_length + thigh_length + torso_length * 2)
        self.left_hand_pos = (0, shoulder_width / -2 - bicep_length
                              - forearm_length, shin_length + thigh_length
                              + torso_length * 2)

        self.right_bicep_pos = (0, shoulder_width / 2, shin_length
                                + thigh_length + torso_length * 2)
        self.right_forearm_pos = (0, shoulder_width / 2 + bicep_length,
                                  shin_length + thigh_length
                                  + torso_length * 2)
        self.right_hand_pos = (0, shoulder_width / 2 + bicep_length
                               + forearm_length, shin_length + thigh_length
                               + torso_length * 2)

        self.head_pos = (0, 0, shin_length + thigh_length + torso_length * 2)

    def calibrate(self):
        self.left_hand = Quaternion(.707, 0, -.707, 0)
        self.left_forearm = Quaternion(.707, 0, -.707, 0)
        self.left_bicep = Quaternion(.707, 0, -.707, 0)

        self.right_hand = Quaternion(.707, 0, .707, 0)
        self.right_forearm = Quaternion(.707, 0, .707, 0)
        self.right_bicep = Quaternion(.707, 0, .707, 0)

        self.left_foot = Quaternion(.707, 0, 0, .707)
        self.left_shin = Quaternion(0, 0, 1, 0)
        self.left_thigh = Quaternion(0, 0, 1, 0)

        self.right_foot = Quaternion(.707, 0, 0, .707)
        self.right_shin = Quaternion(0, 0, 1, 0)
        self.right_thigh = Quaternion(0, 0, 1, 0)

        self.head = Quaternion()
        self.upper_torso = Quaternion()
        self.lower_torso = Quaternion()


'''
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
'''
