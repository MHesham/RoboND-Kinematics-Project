#! /usr/bin/env python

from sympy import *
from mpmath import radians
import tf


def rot_x(q):
    R_x = Matrix([[1,              0,       0, 0],
                  [0,         cos(q), -sin(q), 0],
                  [0,         sin(q),  cos(q), 0],
                  [0,              0,       0, 1]])
    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q),        0,  sin(q), 0],
                  [0,        1,       0, 0],
                  [-sin(q),        0,  cos(q), 0],
                  [0,        0,       0, 1]])
    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q),        0, 0],
                  [sin(q),  cos(q),        0, 0],
                  [0,              0,      1, 0],
                  [0,              0,      0, 1]])
    return R_z


def T_i(alpha, a, theta, d):
    T = Matrix([[cos(theta), -sin(theta), 0, a],
                [sin(theta) * cos(alpha), cos(theta) *
                 cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(theta) * sin(alpha), cos(theta) *
                 sin(alpha),  cos(alpha),  cos(alpha) * d],
                [0, 0, 0, 1]])
    return T


q1, q2, q3, q4, q5, q6, dG = symbols('q1:7,qG')
d1, d2, d3, d4, d5, d6, qG = symbols('d1:7,dG')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
r, p, y = symbols('r p y')
px, py, pz = symbols('px py pz')


class Kr210Solver:
    def __init__(self, DH_params, R_corr, T0_G, R0_3, theta_limits):
        self.DH_params = DH_params
        self.R_corr = R_corr
        self.T0_G = T0_G
        self.R0_3 = R0_3
        self.theta_limits = theta_limits

    def solve_IK(self, ee_x, ee_y, ee_z, roll, pitch, yaw):
        global q1, q2, q3
        global d1, d4, dG
        global a2, a3

        Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * \
            self.R_corr  # x-y-z extrinsic rotation
        EE_0 = Matrix([ee_x, ee_y, ee_z, 1.])
        n_0 = Rrpy[:, 2]
        WC_0 = EE_0 - dG * n_0
        WC_0 = WC_0.subs(self.DH_params)

        theta1 = float(atan2(WC_0[1], WC_0[0]))

        O2_x = a1 * cos(theta1)
        O2_y = a1 * sin(theta1)
        O2_z = d1
        WC_2 = Matrix([WC_0[0] - O2_x, WC_0[1] - O2_y, WC_0[2] - O2_z])
        WC_2 = WC_2.subs(self.DH_params)
        A = sqrt(self.DH_params[d4] ** 2 + self.DH_params[a3] ** 2)
        B = sqrt(WC_2[0] * WC_2[0] + WC_2[1] * WC_2[1] + WC_2[2] * WC_2[2])
        C = self.DH_params[a2]

        phi1 = acos((B * B + C * C - A * A) / (2 * B * C))
        phi2 = asin(WC_2[2] / B)
        phi3 = acos((A * A + C * C - B * B) / (2 ,* A * C))
        phi4 = acos(self.DH_params[d4] / A)

        theta2 = float(pi / 2 - (phi1 + phi2))
        theta3 = float(pi / 2 - (phi3 + phi4))

        R0_3_eval = self.R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6 = R0_3_eval.T * Rrpy[0:3, 0:3]

        # Entries of the rotation matrix
        r21 = R3_6[1, 0]
        r22 = R3_6[1, 1]
        r23 = R3_6[1, 2]
        r13 = R3_6[0, 2]
        r33 = R3_6[2, 2]

        # Euler angles from R3_6 using trig after generating R3_6 as follows:
        # R3_6 = simplify(T3_4 * T4_5 * T5_6)

        # alpha, rotation about z-axis
        theta6 = float(atan2(-r22, r21))
        # beta,  rotation about y-axis
        theta5 = float(atan2(sqrt(r13**2 + r33**2), r23))
        # gamma, rotation about x-axis
        theta4 = float(atan2(r33, -r13))

        return (theta1, theta2, theta3, theta4, theta5, theta6, (WC_0[0], WC_0[1], WC_0[2]))

    def solve_FK(self, theta1, theta2, theta3, theta4, theta5, theta6):
        T0_G_eval = self.T0_G.evalf(
            subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        px = T0_G_eval[0, 3]
        py = T0_G_eval[1, 3]
        pz = T0_G_eval[2, 3]
        return (px, py, pz)


class Kr210SolverEx:
    def __init__(self, DH_params, R_corr, T0_G, R0_3):
        global px, py, pz, r, p, y

        self.DH_params = DH_params
        self.R_corr = R_corr
        self.T0_G = T0_G
        self.R0_3 = R0_3
        self.Rrpy = rot_z(y) * rot_y(p) * rot_x(r) * \
            self.R_corr  # x-y-z extrinsic rotation
        self.EE_0 = Matrix([px, py, pz, 1.])
        n_0 = self.Rrpy[:, 2]
        self.WC_0 = self.EE_0 - dG * n_0
        self.WC_0 = self.WC_0.subs(self.DH_params)
        self.theta1 = atan2(self.WC_0[1], self.WC_0[0])
        O2_x = a1 * cos(self.theta1)
        O2_y = a1 * sin(self.theta1)
        O2_z = d1
        WC_2 = Matrix([self.WC_0[0] - O2_x, self.WC_0[1] -
                       O2_y, self.WC_0[2] - O2_z])
        WC_2 = WC_2.subs(self.DH_params)
        A = sqrt(self.DH_params[d4] ** 2 + self.DH_params[a3] ** 2)
        B = sqrt(WC_2[0] ** 2 + WC_2[1] ** 2 + WC_2[2] ** 2)
        C = self.DH_params[a2]
        phi1 = acos((B * B + C * C - A * A) / (2 * B * C))
        phi2 = asin(WC_2[2] / B)
        phi3 = acos((A * A + C * C - B * B) / (2 * A * C))
        phi4 = acos(self.DH_params[d4] / A)

        self.theta2 = pi / 2 - (phi1 + phi2)
        self.theta3 = pi / 2 - (phi3 + phi4)

        #R0_3_eval = self.R0_3.evalf(subs={q1: theta1_eval, q2: theta2_eval, q3: theta3_eval})
        R3_6 = self.R0_3.T * self.Rrpy[0:3, 0:3]

        # Entries of the rotation matrix
        r21 = R3_6[1, 0]
        r22 = R3_6[1, 1]
        r23 = R3_6[1, 2]
        r13 = R3_6[0, 2]
        r33 = R3_6[2, 2]

        # Euler angles from R3_6 using trig after generating R3_6 as follows:
        # R3_6 = simplify(T3_4 * T4_5 * T5_6)

        # alpha, rotation about z-axis
        self.theta6 = atan2(-r22, r21)
        # beta,  rotation about y-axis
        self.theta5 = atan2(sqrt(r13**2 + r33**2), r23)
        # gamma, rotation about x-axis
        self.theta4 = atan2(r33, -r13)

    def solve_IK(self, ee_x, ee_y, ee_z, roll, pitch, yaw):
        global q1, q2, q3
        global d1, d4, dG
        global a2, a3

        s = {px: ee_x, py: ee_y, pz: ee_z, r: roll, p: pitch, y: yaw}

        theta1_eval = self.theta1.evalf(subs=s)
        theta2_eval = self.theta2.evalf(subs=s)
        theta3_eval = self.theta3.evalf(subs=s)

        theta4_eval = self.theta4.subs(s).evalf(
            subs={q1: theta1_eval, q2: theta2_eval, q3: theta3_eval})
        theta5_eval = self.theta5.subs(s).evalf(
            subs={q1: theta1_eval, q2: theta2_eval, q3: theta3_eval})
        theta6_eval = self.theta6.subs(s).evalf(
            subs={q1: theta1_eval, q2: theta2_eval, q3: theta3_eval})
        WC_0_eval = self.WC_0.evalf(subs=s)

        return (theta1_eval, theta2_eval, theta3_eval, theta4_eval, theta5_eval, theta6_eval, (WC_0_eval[0], WC_0_eval[1], WC_0_eval[2]))

    def solve_FK(self, theta1, theta2, theta3, theta4, theta5, theta6):
        T0_G_eval = self.T0_G.evalf(
            subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        ee_x = T0_G_eval[0, 3]
        ee_y = T0_G_eval[1, 3]
        ee_z = T0_G_eval[2, 3]
        return (ee_x, ee_y, ee_z)


def create_kr210_solver():
    global q1, q2, q3, q4, q5, q6, qG
    global d1, d2, d3, d4, d5, d6, dG
    global a0, a1, a2, a3, a4, a5, a6
    global alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6

    DH_params = {alpha0:       0, a0:      0, d1:  0.75,
                 alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
                 alpha2:       0, a2:   1.25, d3:     0,
                 alpha3: -pi / 2, a3: -0.054, d4:  1.50,
                 alpha4:  pi / 2, a4:      0, d5:     0,
                 alpha5: -pi / 2, a5:      0, d6:     0,
                 alpha6:       0, a6:      0, dG: 0.303, qG: 0}

    theta_limits = {q1: (radians(-185), radians(185)),
                    q2: (radians(-45), radians(85)),
                    q3: (radians(-210), radians(65)),
                    q4: (radians(-350), radians(350)),
                    q5: (radians(-125), radians(125)),
                    q6: (radians(-350), radians(350))}

    T0_1 = T_i(alpha0, a0, q1, d1).subs(DH_params)
    T1_2 = T_i(alpha1, a1, q2, d2).subs(DH_params)
    T2_3 = T_i(alpha2, a2, q3, d3).subs(DH_params)
    T3_4 = T_i(alpha3, a3, q4, d4).subs(DH_params)
    T4_5 = T_i(alpha4, a4, q5, d5).subs(DH_params)
    T5_6 = T_i(alpha5, a5, q6, d6).subs(DH_params)
    T6_G = T_i(alpha6, a6, qG, dG).subs(DH_params)

    R_z = rot_z(pi)
    R_y = rot_y(-pi / 2)
    R_corr = R_z * R_y  # Intrinsic rotation about Z and then Y
    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * R_corr

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]

    return Kr210Solver(DH_params, R_corr, T0_G, R0_3)
