#! /usr/bin/env python

from sympy import *
from mpmath import degrees, radians
import tf
import rospy

# If set to True, prints all possible solutions during the IK step, False
# otherwise.
debug_IK = False


def rot_x(q):
    """Creates a rotation matrix around X axis

    Args:
        q: angle in radiance.

    Returns:
        Matrix: The evaluated rotation matrix Rx.

    """
    R_x = Matrix([[1,              0,       0, 0],
                  [0,         cos(q), -sin(q), 0],
                  [0,         sin(q),  cos(q), 0],
                  [0,              0,       0, 1]])
    return R_x


def rot_y(q):
    """Creates a rotation matrix around Y axis.

    Args:
        q: angle in radiance.

    Returns:
        Matrix: The evaluated rotation matrix Ry.

    """
    R_y = Matrix([[cos(q),         0,  sin(q), 0],
                  [0,              1,       0, 0],
                  [-sin(q),        0,  cos(q), 0],
                  [0,              0,       0, 1]])
    return R_y


def rot_z(q):
    """Creates a rotation matrix around Z axis.

    Args:
        q: angle in radiance.

    Returns:
        Matrix: The evaluated rotation matrix Rz.

    """
    R_z = Matrix([[cos(q),   -sin(q),       0, 0],
                  [sin(q),    cos(q),       0, 0],
                  [0,              0,       1, 0],
                  [0,              0,       0, 1]])
    return R_z


def TF(alpha, a, theta, d):
    """Creates a homogeneous transformation matrix from frame i-1 to frame i
    based on the modified DH convention.

    Args:
        alpha: Twist angle alpha(i - 1) symbol
        a: Link length a(i - 1) symbol
        theta: joint angle theta(i) symbol
        d: link offset d(i) symbol

    Returns:
        Matrix: The homogeneous transformation matrix T(i-1, i) in the symbolic
        form.

    """
    T = Matrix([[cos(theta), -sin(theta), 0, a],
                [sin(theta) * cos(alpha), cos(theta) *
                 cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(theta) * sin(alpha), cos(theta) *
                 sin(alpha),  cos(alpha),  cos(alpha) * d],
                [0, 0, 0, 1]])
    return T


def solve_q4q6(theta5, R3_6):
    """Solves q4 and q6 given a known q5 give that sin(q5) != 0

    Args:
        theta5 (float): q5 angle in radiance.

    Returns:
        q4q6 (float tuble): q4 and q6 angles in radiance.
    """
    # Entries of interest from the R3_6 rotation matrix.
    r21 = R3_6[1, 0]  # sin(q5)cos(q6)
    r22 = R3_6[1, 1]  # -sin(q5)sin(q6)
    r13 = R3_6[0, 2]  # -cos(q4)sin(q5)
    r33 = R3_6[2, 2]  # sin(q4)sin(q5)

    assert sin(theta5) != 0
    if sin(theta5) < 0:
        # atan2(y/x) = atan2(y,x) = atan2(sin,cos)
        # atan2 can result in 4 different solutions for the 4 quadrants
        # (x,y), (-x,y), (-x,-y), (x,-y) i.e atan2(y,x) != atan2(-y,-x)
        # Below we choose to be consistent in the sign of atan2 params
        # when sin(q5) is either positive or negative.
        # Given sin(q5) < 0:
        # r33 =  sin(q4)sin(q5)
        # r13 = -cos(q4)sin(q5)
        # q4 = atan2(-r33, r31)
        # q4 = atan2(-sin(q4)sin(q5), -cos(q4)sin(q5))
        # q4 = atan2(sin(q4), cos(q4))
        theta4 = float(atan2(-r33, r13))
        # r22 = -sin(q5)sin(q6)
        # r21 =  sin(q5)cos(q6)
        # q6 = atan2(r22, -r21)
        # q6 = atan2(-sin(q5)sin(q6), -sin(q5)cos(q6))
        # q6 = atan2(sin(q6), cos(q6))
        theta6 = float(atan2(r22, -r21))
    else:
        # Given sin(q5) > 0:
        # q4 = atan2(r33, -r13)
        # q4 = atan2(sin(q4)sin(q5), cos(q4)sin(q5))
        # q4 = atan2(sin(q4), cos(q4))
        theta4 = float(atan2(r33, -r13))
        # q6 = atan2(-r22, r21)
        # q6 = atan2(sin(q5)sin(q6), sin(q5)cos(q6))
        # q6 = atan2(sin(q6),cos(q6))
        theta6 = float(atan2(-r22, r21))

    return (theta4, theta6)


def solve_q4q5q6(R3_6):
    """Solves q4,q5,q6 given the R3_6 and return multiple solutions if possible.

    Args:
        R3_6 (Matrix): An evaluated R3_6 matrix.

    Returns:
        q4q5q6_solutions (q4q5q6 tuble list): A list of the tuble (q4,q5,q6)
        where each tuble represents a possible solution.
    """
    # Entries of interest from the R3_6 rotation matrix.
    r23 = R3_6[1, 2]  # cos(q5)
    r13 = R3_6[0, 2]  # -cos(q4)sin(q5)
    r32 = R3_6[2, 1]  # sin(q4)cos(q5)sin(q6) - cos(q4)cos(q6)
    r33 = R3_6[2, 2]  # sin(q4)sin(q5)
    q4q5q6_solutions = []

    theta5 = float(atan2(sqrt(r13**2 + r33**2), r23))
    if sin(theta5) == 0:
        # Given q5 = 0, let q4 = 0 and substitute below to get q6:
        # r32 = sin(q4)cos(q5)sin(q6) - cos(q4)cos(q6)
        # r32 = -cos(q6)
        # q6 = acos(-r32)
        theta4 = 0.
        theta6 = float(acos(-r32))
        if not is_angle_within_limits(q6, theta6):
            rospy.logerr(
                "!!singularity detected!! theta6 %04.4f outside joint limits", theta6)
        else:
            rospy.logwarn(
                "!!singularity detected!! theta4 and theta5 are zero, and theta6 %04.4f", theta6)
        q4q5q6_solutions.append([theta4, theta5, theta6])
    else:
        # Taking the positive square root solution
        theta5_sln1 = theta5
        # Taking the negative square root solution
        theta5_sln2 = float(atan2(-sqrt(r13**2 + r33**2), r23))

        theta4_sln1, theta6_sln1 = solve_q4q6(theta5_sln1, R3_6)
        theta4_sln2, theta6_sln2 = solve_q4q6(theta5_sln2, R3_6)

        q4q5q6_solutions.append([theta4_sln1, theta5_sln1, theta6_sln1])
        q4q5q6_solutions.append([theta4_sln2, theta5_sln2, theta6_sln2])

    return q4q5q6_solutions


# Modified DH params table symbols
q1, q2, q3, q4, q5, q6, dG = symbols('q1:7,qG')
d1, d2, d3, d4, d5, d6, qG = symbols('d1:7,dG')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Modified DH params table
DH_params = {alpha0:       0, a0:      0, d1:  0.75,
             alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
             alpha2:       0, a2:   1.25, d3:     0,
             alpha3: -pi / 2, a3: -0.054, d4:  1.50,
             alpha4:  pi / 2, a4:      0, d5:     0,
             alpha5: -pi / 2, a5:      0, d6:     0,
             alpha6:       0, a6:      0, dG: 0.303, qG: 0}

# Angle limits in rad for each joint extracted from the URDF.
# Currently not used, but should be utilized in a future work.
theta_limits = {q1: (radians(-185), radians(185)),
                q2: (radians(-45), radians(85)),
                q3: (radians(-210), radians(65)),
                q4: (radians(-350), radians(350)),
                q5: (radians(-125), radians(125)),
                q6: (radians(-350), radians(350))}

# The homogeneous transforms from frame i-1 to frame i
T0_1 = TF(alpha0, a0, q1, d1).subs(DH_params)
T1_2 = TF(alpha1, a1, q2, d2).subs(DH_params)
T2_3 = TF(alpha2, a2, q3, d3).subs(DH_params)
T3_4 = TF(alpha3, a3, q4, d4).subs(DH_params)
T4_5 = TF(alpha4, a4, q5, d5).subs(DH_params)
T5_6 = TF(alpha5, a5, q6, d6).subs(DH_params)
T6_G = TF(alpha6, a6, qG, dG).subs(DH_params)

# Intrinsic rotation of 180' about the body-fixed Z axis followed by a -90'
# about the body-fixed Y axis.
# R_corr is the rotation of the gripper_link frame in URDF relative to the
# gripper_link DH frame, can be called RG(DH,URDF).
R_z = rot_z(pi)
R_y = rot_y(-pi / 2)
R_corr = R_z * R_y

# The total homogeneous transformation between frame 0 and griper frame and
# is post-multiplied by R_corr so that input poses for the gripper_link from
# ROS are transformed into the DH frame.
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * R_corr

# The rotation matrix between frame 0 and frame 3.angle
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]


def is_angle_within_limits(q, angle_rad):
    """Check if a revolute joint angle is within URDF defined limits.

    Args:
        q (symbol): sympy angle symbol.
        angle_rad (float): angle to validate in radiance

    Returns:
        True if angle is within limits, False otherwise. 
    """
    if angle_rad >= theta_limits[q][0] and \
            angle_rad <= theta_limits[q][1]:
        return True
    return False


class Kr210SolverFactory(object):
    """A factory class for creating instances of KR210 solvers

    """
    @staticmethod
    def create(solver_name):
        """Factory method that creates objects of KR210 IK solvers.

        Args:
            solver_name (string): The solver class name.

        Returns:
            An object of the solver requested if name is valid, otherwise None.

        """
        if solver_name == "Kr210Solver":
            return Kr210Solver()
        elif solver_name == "Kr210LazySolver":
            return Kr210LazySolver()
        else:
            return None


class Kr210Solver(object):
    """An IK and FK solver for KR210 based on the modified DH convetion.
    This solver is the first approach to solving the IK problem and is considered
    the base implementation among other KR210 solvers.

    """

    def solve_IK(self, ee_x, ee_y, ee_z, roll, pitch, yaw):
        """Solve the IK problem given the end-effector (EE) pose relative to
        the base_link.

        Args:
            ee_x (float): The EE X position.
            ee_y (float): The EE Y position.
            ee_z (float): The EE Z position.
            roll (float): The EE Roll around the X axis.
            pitch (float): The EE Pitch around the Y axis.
            yaw (float): The EE Yaw around the Z axis.

        Returns:
            thetas (float list): A list of joint angles theta1 through theta6,
            where the angle at list[0] is theta1.
            wrist center (float tuble): The x,y,z position of the wrist center
            relative to the base_link.

        """
        # Rrpy, the EE rotation matrix from an x-y-z extrinsic rotation post
        # multiplied by the R_corr.
        self.Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
        # The EE position vector relative to base_link.
        EE_0 = Matrix([ee_x, ee_y, ee_z, 1.])

        #
        # Inverse Position Kinematics
        #

        # The EE body-fixed Z axis.
        n_0 = self.Rrpy[:, 2]
        # Wrist center position relative to base_link.
        WC_0 = EE_0 - dG * n_0
        WC_0 = WC_0.subs(DH_params)

        theta1 = float(atan2(WC_0[1], WC_0[0]))

        # Compute frame 2 position relative to base_link.
        O2_x = a1 * cos(theta1)
        O2_y = a1 * sin(theta1)
        O2_z = d1
        # Transform wrist center to be relative to frame 2 instead.
        WC_2 = Matrix([WC_0[0] - O2_x, WC_0[1] - O2_y, WC_0[2] - O2_z])
        WC_2 = WC_2.subs(DH_params)
        # Compute the 3 sides of our SSS triangle.
        A = sqrt(DH_params[d4] ** 2 + DH_params[a3] ** 2)
        B = sqrt(WC_2[0] * WC_2[0] + WC_2[1] * WC_2[1] + WC_2[2] * WC_2[2])
        C = DH_params[a2]

        # Compute intermediate angles from the SSS triangle.
        phi1 = acos((B * B + C * C - A * A) / (2 * B * C))
        phi2 = asin(WC_2[2] / B)
        phi3 = acos((A * A + C * C - B * B) / (2 * A * C))
        phi4 = acos(DH_params[d4] / A)

        theta2 = float(pi / 2 - (phi1 + phi2))
        theta3 = float(pi / 2 - (phi3 + phi4))

        #
        # Inverse Orientation Kinematics
        #

        # Evaluate R0_3 and use to compute R3_6.
        R0_3_eval = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6 = R0_3_eval.T * self.Rrpy[0:3, 0:3]

        # Euler angles from R3_6 using trig after generating R3_6 as follows:
        # R3_6 = simplify(T3_4 * T4_5 * T5_6)

        q4q5q6_solutions = solve_q4q5q6(R3_6)
        theta4, theta5, theta6 = self.choose_best_q4q5q6(q4q5q6_solutions)

        if not is_angle_within_limits(q4, theta4):
            rospy.logwarn("theta4 %04.4f is out of joint limits", theta4)

        if not is_angle_within_limits(q5, theta5):
            rospy.logwarn("theta5 %04.4f is out of joint limits", theta5)

        if not is_angle_within_limits(q6, theta6):
            rospy.logwarn("theta6 %04.4f is out of joint limits", theta6)

        return ([theta1, theta2, theta3, theta4, theta5, theta6], [WC_0[0], WC_0[1], WC_0[2]])

    def solve_FK(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """Solve the FK problem given the 6 joint angles.

        Args:
            theta1 to theta6: The 6 revoluate joint angles in rad.

        Returns:
            EE position (list float): The x,y,z of the EE relative to base_link.
        """
        # The FK solution is simply the evaluation of the total transform T0_G
        # using the 6 joint angles.
        T0_G_eval = T0_G.evalf(
            subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        fk_px = T0_G_eval[0, 3]
        fk_py = T0_G_eval[1, 3]
        fk_pz = T0_G_eval[2, 3]
        return [fk_px, fk_py, fk_pz]

    def choose_best_q4q5q6(self, q4q5q6_solutions):
        """
        """
        assert len(q4q5q6_solutions) > 0

        if debug_IK == True:
            rospy.loginfo("Choosing from %d solution(s)", len(q4q5q6_solutions))
            for i in xrange(0, len(q4q5q6_solutions)):
                sln = q4q5q6_solutions[i]
                rospy.loginfo("  sln#%d: %04.4f' %04.4f' %04.4f'", i + 1,
                            degrees(sln[0]), degrees(sln[1]), degrees(sln[2]))

        # TODO: Implement logic to select best solution from multiple solutions
        # based on current joint states. For now just pick the first solution.
        theta4 = q4q5q6_solutions[0][0]
        theta5 = q4q5q6_solutions[0][1]
        theta6 = q4q5q6_solutions[0][2]
        return (theta4, theta5, theta6)

r, p, y = symbols('r p y')
px, py, pz = symbols('px py pz')


class Kr210LazySolver(object):
    """An IK and FK solver for KR210 based on the modified DH convetion.
    This solver is an experimental approach to solving the IK problem by trying
    to delay all symbol evaluations to the solve step.
    Each of the theta angles thus become a huge expression parameterized by
    only the EE pose.

    """

    def __init__(self):
        self.Rrpy = rot_z(y) * rot_y(p) * rot_x(r) * \
            R_corr  # x-y-z extrinsic rotation
        self.EE_0 = Matrix([px, py, pz, 1.])

        #
        # Inverse Position Kinematics
        #

        n_0 = self.Rrpy[:, 2]
        self.WC_0 = self.EE_0 - dG * n_0
        self.WC_0 = self.WC_0.subs(DH_params)
        self.theta1 = atan2(self.WC_0[1], self.WC_0[0])
        O2_x = a1 * cos(self.theta1)
        O2_y = a1 * sin(self.theta1)
        O2_z = d1
        WC_2 = Matrix([self.WC_0[0] - O2_x, self.WC_0[1] -
                       O2_y, self.WC_0[2] - O2_z])
        WC_2 = WC_2.subs(DH_params)
        A = sqrt(DH_params[d4] ** 2 + DH_params[a3] ** 2)
        B = sqrt(WC_2[0] ** 2 + WC_2[1] ** 2 + WC_2[2] ** 2)
        C = DH_params[a2]
        phi1 = acos((B * B + C * C - A * A) / (2 * B * C))
        phi2 = asin(WC_2[2] / B)
        phi3 = acos((A * A + C * C - B * B) / (2 * A * C))
        phi4 = acos(DH_params[d4] / A)

        self.theta2 = pi / 2 - (phi1 + phi2)
        self.theta3 = pi / 2 - (phi3 + phi4)

        #
        # Inverse Orientation Kinematics
        #

        R3_6 = R0_3.T * self.Rrpy[0:3, 0:3]

        # Entries of the rotation matrix
        r21 = R3_6[1, 0]
        r22 = R3_6[1, 1]
        r23 = R3_6[1, 2]
        r13 = R3_6[0, 2]
        r33 = R3_6[2, 2]

        # Euler angles from R3_6 using trig after generating R3_6 as follows:
        # R3_6 = simplify(T3_4 * T4_5 * T5_6)

        self.theta6 = atan2(-r22, r21)
        self.theta5 = atan2(sqrt(r13**2 + r33**2), r23)
        self.theta4 = atan2(r33, -r13)

    def solve_IK(self, ee_x, ee_y, ee_z, roll, pitch, yaw):
        """Solve the IK problem given the end-effector (EE) pose relative to
        the base_link.

        Args:
            ee_x (float): The EE X position.
            ee_y (float): The EE Y position.
            ee_z (float): The EE Z position.
            roll (float): The EE Roll around the X axis.
            pitch (float): The EE Pitch around the Y axis.
            yaw (float): The EE Yaw around the Z axis.

        Returns:
            thetas (float list): A list of joint angles theta1 through theta6,
            where the angle at list[0] is theta1.
            wrist center (float tuble): The x,y,z position of the wrist center
            relative to the base_link.

        """
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

        return ([theta1_eval, theta2_eval, theta3_eval, theta4_eval, theta5_eval, theta6_eval], [WC_0_eval[0], WC_0_eval[1], WC_0_eval[2]])

    def solve_FK(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """Solve the FK problem given the 6 joint angles.

        Args:
            theta1 to theta6: The 6 revoluate joint angles in rad.

        Returns:
            EE position (list float): The x,y,z of the EE relative to base_link.
        """
        T0_G_eval = T0_G.evalf(
            subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        ee_x = T0_G_eval[0, 3]
        ee_y = T0_G_eval[1, 3]
        ee_z = T0_G_eval[2, 3]
        return [ee_x, ee_y, ee_z]
