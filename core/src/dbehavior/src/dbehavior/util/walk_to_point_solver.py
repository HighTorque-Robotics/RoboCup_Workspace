from dbehavior.util import VecPos
from dbehavior.util.mathutil import (sign, cosd, sind,
                                     deg2rad, rad2deg)


class WalkToPointConfig(object):
    """
    Config for ``WalkToPointSolver``.
    """

    def __init__(self, param=None):
        """
        Initialize ``WalkToPointConfig``.

        :param param: parameters from ROS
        :type param: Parameter
        """
        if param:
            self.NEARBY_SWITCHING_DISTANCE = param.nearby_switching_distance
            self.FARAWAY_SWITCHING_DISTANCE = param.faraway_switching_distance
            self.TOP_X_MAX = param.top_x_max
            self.TOP_THETA_MAX = param.top_theta_max
            self.MID_X_MAX = param.mid_x_max
            self.MID_THETA_MAX = param.mid_theta_max
            self.AD_X_MAX = param.ad_x_max
            self.AD_THETA_MAX = param.ad_theta_max
            self.STEP_THETA_MAX = param.step_theta_max
            self.THETA_SAFE = param.theta_safe
            self.epso = param.epso
        else:
            self.NEARBY_SWITCHING_DISTANCE = 30
            self.FARAWAY_SWITCHING_DISTANCE = 40
            self.TOP_X_MAX = 6.0
            self.TOP_THETA_MAX = 5.0
            self.MID_X_MAX = 4.5
            self.MID_THETA_MAX = 10.0
            self.AD_X_MAX = 2.0
            self.AD_THETA_MAX = 10.0
            self.STEP_THETA_MAX = 15.0
            self.THETA_SAFE = 10.0
            self.epso = 1e-10

    @property
    def walk_ability(self):
        return [[self.TOP_X_MAX, self.TOP_X_MAX, self.MID_X_MAX, self.AD_X_MAX, self.epso],
                [self.epso, self.TOP_THETA_MAX, self.MID_THETA_MAX, self.AD_THETA_MAX, self.STEP_THETA_MAX]]

    @property
    def walk_ability_length(self):
        return len(self.walk_ability[0])


class WalkToPointSolver(object):
    """
    Solver for walking to destination point. Input must be in the same coordinate,
    For simplicity, we use destination in local coordinate system for calculating.

    Refer to documentation of Dongdong Yu, if you get in trouble,
    just call him: 18768116076.
    """

    def __init__(self, param=None):
        """
        Initialize ``WalkToPointSolver``.
        """
        self.config = WalkToPointConfig(param)
        self.near_distance = self.config.NEARBY_SWITCHING_DISTANCE
        self.vx = 0

    def solve_field(self, dst, vx, force_omni=False):
        """
        Solve for destination position in robot coordinate.

        :param dst: destination position
        :type dst: VecPos
        :param vx: robotCtrl.x in MotionInfo
        :type vx: float
        :param force_omni: whether or not to force to use omni method
        :type force_omni: bool
        :return: walk command [forward, left, turn]
        :rtype: list
        """
        self.vx = vx

        if dst.length() < self.near_distance or force_omni:
            self.near_distance = self.config.FARAWAY_SWITCHING_DISTANCE
            return self.omni_direction(dst)
        else:
            self.near_distance = self.config.NEARBY_SWITCHING_DISTANCE
            return self.fast(dst)

    def solve_global(self, dst_global, robot_pos, vx, force_omni=False):
        """
        Solve for destination position in field coordinate.

        :param dst_global: destination position in global coordinate
        :type dst_global: VecPos
        :param robot_pos: robot position in field coordinate
        :type robot_pos: VecPos
        :param vx: robotCtrl.x in MotionInfo
        :type vx: float
        :param force_omni: whether or not to force to use omni method
        :type force_omni: bool
        :return: walk command [forward, left, turn]
        :rtype: list
        """
        dest = VecPos.calc_field_position(dst_global, robot_pos)
        return self.solve_field(dest, vx, force_omni)

    def omni_direction(self, local_dst):
        """
        Walk to destination with omni direction.

        :param local_dst: nearby destination
        :type local_dst: VecPos
        :return: walk command [forward, left, turn]
        :rtype: list
        """
        mirror = False
        dst = local_dst.copy()

        if dst.y < 0:
            dst.y = -dst.y
            dst.z = -dst.z
            mirror = True

        length = min(self.config.AD_X_MAX, dst.length())
        theta_p = dst.slope()
        theta_attack = dst.z

        if abs(theta_attack) > self.config.AD_THETA_MAX:
            theta_out = self.config.AD_THETA_MAX * sign(theta_attack)
        else:
            theta_out = theta_attack

        tmp_angle = theta_p - theta_out
        x_out = length * cosd(tmp_angle)
        y_out = length * sind(tmp_angle)

        if mirror:
            y_out = -y_out
            theta_out = -theta_out

        return x_out, y_out, theta_out

    def fast(self, local_dst):
        """
        Walk to far destination fast.

        :param local_dst: nearby destination
        :type local_dst: VecPos
        :return: walk command [forward, left, turn]
        :rtype: list
        """
        sx_star, sy_star, st_star = 0, 0, 0
        mirror = False
        dest = local_dst.copy()

        if dest.y < 0:
            dest.y = -dest.y
            dest.z = -dest.z
            mirror = True

        x_input = self.vx
        theta_p = dest.slope()
        theta_attack = dest.z

        theta_safe = min(theta_p, self.config.THETA_SAFE)
        theta_attack_p = theta_attack - theta_p

        a_dest = dest.copy()
        a_dest.rotate(90)

        if theta_attack_p > theta_p:
            theta_max = theta_p
        elif theta_attack_p > theta_safe:
            theta_max = theta_attack_p
        else:
            theta_max = theta_safe

        theta_max = min(theta_max, theta_safe)
        theta_max = min(theta_max, theta_p)

        dt = 1
        stepnum_min = 999999

        i = 0
        if abs(theta_max) < self.config.TOP_THETA_MAX:
            sx_star, sy_star, st_star = self.config.TOP_X_MAX, 0, theta_max
        else:
            cross_i = VecPos()
            while i < theta_max:
                a_i = a_dest.copy()
                a_i.rotate(i)
                b_i = a_i.dot(dest)
                r_i = b_i / (a_i.y - a_i.length() + self.config.epso)

                sx, sy, st = self.getx_max(r_i)
                st = min(theta_p, st)
                c_i = i + theta_p
                cross_i.x = r_i * sind(c_i)
                cross_i.y = r_i * (1 - cosd(c_i))
                line_i = (dest - cross_i).copy()

                k1 = 15.0
                k2 = 10.0
                stepnum_arc = c_i / (st == 0 and self.config.epso or st)
                stepnum_line = line_i.length() / self.config.TOP_X_MAX
                if theta_p < self.config.AD_THETA_MAX + 20:
                    k2 = 25.0

                stepnum_speed = abs(max(-sx + x_input, 0)) / self.config.TOP_X_MAX * k1 * deg2rad(theta_p) + \
                                abs(self.config.TOP_X_MAX - sx) * k2 / self.config.TOP_X_MAX

                stepnum = stepnum_arc + stepnum_line + stepnum_speed
                if stepnum_min > stepnum:
                    stepnum_min = stepnum
                    sx_star, sy_star, st_star = sx, sy, st

                i += dt

        if mirror:
            st_star = -st_star
            # sy_star = -sy_star

        # print 'fast', sx_star, sy_star, st_star, localDest.x, localDest.y, localDest.z
        return sx_star, sy_star, st_star

    def getx_max(self, r):
        """
        Get max sx and max st

        :param r:
        :type r:
        :return:
        :rtype:
        """
        a_norm = [[0 for col in range(0, self.config.walk_ability_length - 1)]
                  for row in range(0, self.config.walk_ability_length)]
        b = [0 for col in range(0, self.config.walk_ability_length - 1)]

        for i in range(0, 4):
            a_norm[1][i] = self.config.walk_ability[1][i + 1] - self.config.walk_ability[1][i] + self.config.epso
            a_norm[0][i] = -self.config.walk_ability[0][i + 1] + self.config.walk_ability[0][i] + self.config.epso
            b[i] = a_norm[0][i] * self.config.walk_ability[1][i] + a_norm[1][i] * self.config.walk_ability[0][i]
        rec = [rad2deg(self.config.walk_ability[0][i] * 1.0 / self.config.walk_ability[1][i]) for i in
               range(0, len(self.config.walk_ability[0]))]

        # FIXME check this initialization is correct
        gait_sx = 0
        gait_st = 0
        for i in range(0, 4):
            if r <= rec[i]:
                if abs(a_norm[1][i]) < self.config.epso:
                    gait_sx = deg2rad(r) * self.config.walk_ability[1][i]
                    gait_st = self.config.walk_ability[1][i]
                else:
                    gait_st = b[i] / a_norm[1][i] / \
                              (deg2rad(r) + a_norm[0][i] / a_norm[1][i])
                    gait_sx = (b[i] - a_norm[0][i] * gait_st) / a_norm[1][i]

        gait_sy = 0
        return gait_sx, gait_sy, gait_st
