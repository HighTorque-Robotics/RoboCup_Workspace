from dbehavior.util import VecPos
from dbehavior.util.mathutil import PI, degree_between, get_dis, angle_between, degrees
from math import sin, cos


class AttackPointSolverConfig(object):
    """
    Config for ``AttackPointSolver``.
    """

    def __init__(self):
        """
        Initialize ``AttackPointSolver``.
        """
        self.MAGIC = 40
        self.DOGE_POINT = VecPos(self.MAGIC, 0)
        self.DOGE_POINT_UP = VecPos(self.MAGIC, self.MAGIC)
        self.DOGE_POINT_DOWN = VecPos(self.MAGIC, -self.MAGIC)
        self.DOGE_ANGLE = degree_between(self.DOGE_POINT, self.DOGE_POINT_UP)


class AttackPointSolver(object):

    def __init__(self):
        self.rub = None
        self.final_dest = None
        self.dest = None
        self.config = AttackPointSolverConfig()

    def calc_attack_point(self, bb):
        """
        :param bb: blackboard
        :type bb: DBlackboard
        :return: final_dest, dest, rub
        :rtype: tuple
        """

        robot_pos = VecPos.from_vector3(bb.vision_info.robot_pos)
        ball_global = VecPos.from_vector3(bb.vision_info.ball_global)
        ball_field = VecPos.from_vector3(bb.vision_info.ball_field)
        attack_target = bb.attack_target

        self.final_dest = self.calc_final_dest(ball_field, ball_global, bb)
        self.rub = self.calc_rub(ball_global, attack_target, robot_pos)
        theta = degree_between(self.config.DOGE_POINT, self.rub)

        if self.rub.x < -10:
            if theta > self.config.DOGE_ANGLE:
                self.dest = self.calc_doge_point(robot_pos, ball_global, attack_target, 'up')
            else:
                self.dest = self.calc_doge_point(robot_pos, ball_global, attack_target, 'down')
        else:
            self.dest = self.final_dest

        if get_dis(robot_pos, self.dest) < 40 or get_dis(robot_pos, self.final_dest) < 40:
            self.dest = self.final_dest

        return self.final_dest, self.dest, self.rub

    @staticmethod
    def calc_final_dest(ball_field, ball_global, bb):
        theta = angle_between(bb.attack_target, ball_global)

        closer_to_left_foot = ball_field.y > 0

        # FIXME(MWX): may blur
        if not bb.kick_enabled or (not bb.left_kick_enabled and not bb.right_kick_enabled):
            kick_point = bb.param.kick_point_left \
                if closer_to_left_foot else bb.param.kick_point_right
            if closer_to_left_foot:
                bb.left_kick_enabled = True
                bb.right_kick_enabled = False
            else:
                bb.right_kick_enabled = True
                bb.left_kick_enabled = False
            bb.kick_direction_decided = True
        else:
            kick_point = bb.param.kick_point_left \
                if bb.left_kick_enabled else bb.param.kick_point_right

        res = VecPos()
        res.x = ball_global.x + kick_point.x * cos(theta)
        res.y = ball_global.y + kick_point.x * sin(theta)

        theta2 = angle_between(bb.attack_target, res)
        res.z = degrees(theta2 + PI)
        return res

    @staticmethod
    def calc_rub(ball_global, target, robot_pos):
        b2r = robot_pos - ball_global
        theta = degree_between(target, ball_global)
        b2r.rotate(-theta)
        return b2r

    def calc_doge_point(self, robot_pos, ball_global, target, side):
        doge = side is 'up' and self.config.DOGE_POINT_UP or self.config.DOGE_POINT_DOWN

        if -30 < self.rub.x < 0 and abs(self.rub.y) < 20:
            doge = side is 'up' and VecPos(0, 20) or VecPos(0, -20)

        theta = angle_between(ball_global, target)
        dogex = ball_global.x + doge.y * sin(theta) - doge.x * cos(theta)
        dogey = ball_global.y - doge.y * cos(theta) + doge.x * sin(theta)
        d = VecPos(dogex, dogey)
        angle = degree_between(d, self.final_dest)

        return VecPos(dogex, dogey, angle)
