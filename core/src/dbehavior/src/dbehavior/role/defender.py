from math import fabs
from dbehavior.btree.leaf import ConditionLeaf
from dbehavior.btree.branch import (DynamicGuardSelector, Parallel,
                                    GuardSelector, GuardSequence)
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  ReEntryOccurred, ReEntryLocalization,
                                  GotInitPos, GotOutOfInitPos, BallSeen,
                                  MotionReStable)
from dbehavior.btree.decorator import Invert
from dbehavior.btree.core import Status
from dbehavior.btree.leaf import ConditionLeaf
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  ReEntryOccurred, ReEntryLocalization,
                                  GotInitPos, GotOutOfInitPos, BallSeen,
                                  TeamBallSeen, MotionReStable, Kicking)
from dbehavior.core import Role, Skill, Do
from dbehavior.skill import (PrepareIMU, Attack, ScanField, ScanFieldNew,
                             FindBall, SeekBall, SearchBall, TrackBall,
                             InitialParticles, GoToInitPos, EntryScanField,
                             ScanFieldDown)
from dbehavior.util import VecPos
from dbehavior.util.mathutil import degree_between, get_magnitude


class BallInDefendZone(ConditionLeaf):

    def __init__(self):
        super(BallInDefendZone, self).__init__()
        self.defend_radius = 150
        self.half_field_tol = 120

    def condition(self):
        see_ball = self.get_bb().vision_info.see_ball or self.get_bb(
        ).team_ball_seen
        if see_ball:
            ball_global = self.get_bb().ball_global
            ball_field = self.get_bb().ball_field
            ball_in_our_half = (self.get_bb().param.attack_right and ball_global.x < -self.half_field_tol) or \
                               (not self.get_bb().param.attack_right \
                                 and ball_global.x > self.half_field_tol)
            close_to_ball = get_magnitude(ball_field) <= self.defend_radius
            return ball_in_our_half or close_to_ball
        else:
            return False
        # if ball is seen, and it's in our half field or near the robot
        # it's in our defending zone
        ball_global = self.get_bb().ball_global
        ball_field = self.get_bb().ball_field
        ball_in_our_half = (self.get_bb().param.attack_right and ball_global.x < self.half_field_tol) or \
                           (not self.get_bb().param.attack_right and ball_global.x > self.half_field_tol)
        close_to_ball = get_magnitude(ball_field) <= self.defend_radius
        return ball_in_our_half or close_to_ball


class BallOutOfDefendZone(ConditionLeaf):

    def __init__(self):
        super(BallOutOfDefendZone, self).__init__()
        self.defend_radius_hys = 200
        self.half_field_tol = 50

    def condition(self):
        see_ball = self.get_bb().vision_info.see_ball or self.get_bb(
        ).team_ball_seen
        if see_ball:
            ball_global = self.get_bb().ball_global
            ball_field = self.get_bb().ball_field
            ball_in_enemy_half = (self.get_bb().param.attack_right and ball_global.x > self.half_field_tol) or \
                                 (not self.get_bb(
                                 ).param.attack_right and ball_global.x < -self.half_field_tol)
            far_from_ball = get_magnitude(ball_field) > self.defend_radius_hys
            return ball_in_enemy_half and far_from_ball
        else:
            return False
        # if ball is seen, and it's out of our half field and far away from the robot
        # it's out of our defending zone
        ball_global = self.get_bb().ball_global
        ball_field = self.get_bb().ball_field
        ball_in_enemy_half = (self.get_bb().param.attack_right and ball_global.x > self.half_field_tol) or \
                             (not self.get_bb().param.attack_right and ball_global.x < -self.half_field_tol)
        far_from_ball = get_magnitude(ball_field) > self.defend_radius_hys
        return ball_in_enemy_half and far_from_ball


class DefenderBrain(Skill):
    """
    Determine attack target.
    """

    def __init__(self, bb):
        super(DefenderBrain, self).__init__(bb)
        self.defend_radius = 75
        self.default_attack_target = bb.attack_target
        self.magic_y = 30
        if bb.param.attack_right:
            self.target = VecPos(550, self.magic_y)
        else:
            self.target = VecPos(-550, self.magic_y)

    def execute(self):
        if self.bb.ball_global is not None:
            ball_global = VecPos.from_vector3(self.bb.ball_global)
            ball_field = VecPos.from_vector3(self.bb.ball_field)
            robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
            # NOTE: find appropriate threshold
            if fabs(ball_global.x) < fabs(
                    robot_pos.x) and get_magnitude(ball_field) > 30:
                r2b = ball_global - robot_pos
                target = ball_global + r2b.normalize(self.defend_radius)
                target.z = degree_between(ball_global, robot_pos)
                self.bb.attack_target = target
            else:
                if self.bb.param.attack_right:
                    self.bb.attack_target = VecPos(550, 0)
                else:
                    self.bb.attack_target = VecPos(-550, 0)
        else:
            if self.bb.param.attack_right:
                self.bb.attack_target = VecPos(550, 0)
            else:
                self.bb.attack_target = VecPos(-550, 0)

        self.bb.kick_enabled = True
        return Status.RUNNING


class DefendBall(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (Attack(bb),
             BallInDefendZone(), BallOutOfDefendZone()),
             # TODO: Align to ball
        ]
        super(DefendBall, self).__init__(children, buffer_size=5, use_exit=True)


class Defend(Parallel):

    def __init__(self, bb):
        args = (DefenderBrain(bb), DynamicGuardSelector([
            # Robot look ahead when kicking[written in Attack()]
            (Attack(bb), Kicking()),
            (Parallel(
                DefendBall(bb),
                TrackBall(bb)), BallSeen()),
            # TODO: Turn to ball and find ball if TeamBallSeen()
            (SeekBall(bb), None)
        ]))
        super(Defend, self).__init__(*args)


class SimpleDefendBall(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (DynamicGuardSelector([
                (Attack(bb), GuardSelector(BallSeen(), TeamBallSeen(),
                                           Kicking())),
                (SeekBall(bb), None),
            ]), BallInDefendZone(), Invert(BallInDefendZone())),
            (Parallel(
                DynamicGuardSelector([(TrackBall(bb), BallSeen()),
                                      (ScanFieldDown(bb), None)]),
                GoToInitPos(bb)), 
             GotOutOfInitPos(dis_tol=100, angle_tol=50),
             GuardSelector(BallInDefendZone(), GotInitPos(dis_tol=30))),
            (TrackBall(bb), BallSeen(), None),
            (ScanFieldDown(bb), None, None),
        ]
        super(SimpleDefendBall, self).__init__(children,
                                               buffer_size=5,
                                               use_exit=True)


class SimpleDefend(Parallel):

    def __init__(self, bb):
        args = (DefenderBrain(bb), SimpleDefendBall(bb))
        super(SimpleDefend, self).__init__(*args)


class Defender(Role):
    """
    A defender is positioning itself not too far away from the goal
    in order to defend the goal against opponent strikers.
    """

    def __init__(self, bb):
        super(Defender, self).__init__(bb)
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb),
                      InitialParticles(bb)), LowerBoardReconnected()),
            # (Do(bb, cmd='look'), IMUInitialized()),
            (EntryScanField(bb),
             GuardSelector(IMUInitialized(), ReEntryLocalization())),
            (Parallel(GoToInitPos(bb), FindBall(bb)),
             GuardSequence(ReEntryOccurred(), Invert(BallSeen()))),
            (ScanFieldNew(bb), MotionReStable()),
            (SimpleDefend(bb), None)
        ])

        self.add_child(root)
