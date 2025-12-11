from collections import deque
from math import fabs, sqrt

from dbehavior.btree.branch import Parallel, DynamicGuardSelector, GuardSequence, GuardSelector
from dbehavior.btree.core import Status
from dbehavior.btree.decorator import Invert
from dbehavior.btree.leaf import (ConditionLeaf, LowerBoardReconnected,
                                  IMUInitialized, ReEntryOccurred,
                                  ReEntryLocalization, BallSeen, TeamBallSeen,
                                  GotInitPos, GotOutOfInitPos, GotPos,
                                  MotionReStable, Kicking)
from dbehavior.core import Skill, Role, Do
from dbehavior.skill import (PrepareIMU, ScanField, ScanFieldNew, ScanFieldDown,
                             SeekBall, FindBall, TrackBall, Attack,
                             InitialParticles, GoToInitPos, ScanField)
from dbehavior.util import VecPos, Timer
from dbehavior.util.mathutil import degree_between, get_magnitude, get_dis, abs_angle_diff
from dmsgs.msg import BodyCommand
from dbehavior.skill.entry import Walk
"""
Ball velocity high(in danger): Save ball.
                    (far)   Align
Ball velocity low: {
                    (align) Attack
"""


class GoalKeeper(Role):
    """
    A goalie is positioned inside the penalty box, and the last line
    of defense. It may touch the ball, and it has special protection.
    """

    def __init__(self, bb):
        super(GoalKeeper, self).__init__(bb)
        # yapf: disable
        root = DynamicGuardSelector([
            (Parallel(
                PrepareIMU(bb),
                InitialParticles(bb)), LowerBoardReconnected()),
            (DynamicGuardSelector([
                # (SaveBall(bb), GuardSequence(BallInDanger(), GoalieReadyToSave()), None),
                (Parallel(
                    Attack(bb),
                    GoalKeeperBrain(bb)),
                 GuardSelector(BallInGoalieAttackZone(), Kicking()),
                 GuardSequence(BallOutOfGoalieAttackZone(), Invert(Kicking()))),
                (Parallel(
                    TrackBall(bb, speed=1),
                    GoalKeeperStayHome(bb)), None, None)
            ], use_exit=True), BallSeen()),
            (Parallel(
                GoalKeeperScan(bb),
                GoalKeeperStayHome(bb)), None)
        ])
        # yapf: enable
        self.add_child(root)


class GoalKeeperBase():
    """
    A goal keeper is positioned inside the penalty zone, and the last line
    of defense. It may touch the ball, and it has special protection.
    """

    def __init__(self):
        self.get_attr()
        # super(GoalKeeperBase, self).__init__(bb)

    def get_attr(self):
        # if ball in margin position regarding to goal area, then attack [cm]
        self.attack_margin = VecPos(50, 130)
        # attack margin hysteresis (value added to attack margin) [cm]
        self.attack_margin_hys = VecPos(25, 15)

        self.home_pos = None
        # distance to goal line for waiting [cm]
        self.home_x = 50
        self.home_y = 0
        # distance to home position acceptable for waiting
        # (in front of home_x line) [cm]
        self.home_dis_max = 50
        self.home_angle_max = 30
        # hysteresis distance to home position acceptable for waiting [cm]
        self.home_dis_max_hys = 50
        self.home_angle_max_hys = 25

        # length of the buffer used to smooth next state change
        self.next_state_size = 5
        self.next_state = deque(maxlen=self.next_state_size)

        # ignore ball if ball is out of this position [cm]
        self.ball_ignore_x = 450
        # hysteresis distance to ignore ball [cm]
        self.ball_ignore_x_hys = 50

        # consider is align is distance with optimal point is below
        # this value (is added to placer tolerance) [deg]
        self.align_tol = 10

        # Enemy's adversary maximum shoot distance [cm]
        self.kick_dis_max = 300

        # target [cm]
        self.target = VecPos()
        # target smoothing coffecient
        self.target_smoothing_coff = 0.95

        # FIXME from placer
        self.margin_xy = 50

        self.state = 'start_wait'
        self.timer = Timer()
        self.home_pos = None

    def buffered_set_state(self, state):
        self.next_state.append(state)
        ok = True
        for s in self.next_state:
            if s != self.next_state[0]:
                ok = False
        if ok:
            self.state = state


class SaveBall(Skill):

    def __init__(self, bb):
        super(SaveBall, self).__init__(bb)

    # TODO: More accurate ball model
    def execute(self):
        ballv = self.bb.vision_info.ball_velocity
        ball = self.bb.vision_info.ball_field
        if ballv.x > -15:
            return Status.RUNNING
        last_y = ball.y - (ballv.y) / ballv.x * ball.x
        if last_y > 2000:
            self.goalie_left()
            self.bb.falled = True
        elif last_y < -2000:
            self.goalie_right()
            self.bb.falled = True
        else:
            self.bb.action_cmd.bodyCmd = self.action_generator.goalie_mid()
        return Status.RUNNING


class BallInDanger(ConditionLeaf):
    '''
    Save ball when it's in a high speed
    '''

    def __init__(self):
        super(BallInDanger, self).__init__()
        # Adversary acceleration for ball rolling on the grass.
        # The lower, the earlier.
        # TODO: Take it as a param and test for every field
        self.adv_acc = 70

        # Reaction time from sending command to lying down
        self.reac_time = 2

    def condition(self):
        if not self.get_bb().vision_info.see_ball:
            return False
        ball_dist = abs(self.get_bb().vision_info.ball_field.x)
        ball_v = self.get_bb().vision_info.ball_velocity.x
        if ball_v > -30:
            return False
        ball_v = abs(ball_v)
        if ball_v / self.adv_acc < self.reac_time:
            adv_dist = ball_v**2 / (2 * self.adv_acc)
        else:
            adv_dist = ball_v * self.reac_time - \
                self.adv_acc * self.reac_time * self.reac_time / 2
        # print('ball_v: {} adv_dist: {} ball_dist: {}'.format(
        #     ball_v, adv_dist, ball_dist))
        if adv_dist > ball_dist:
            return True
        else:
            return False


class GoalKeeperScan(ScanField):

    def __init__(self, bb, timeout=1):
        super(GoalKeeperScan, self).__init__(bb, timeout=0)
        self.pitch_speed = 1
        self.gaze_plats = []
        for i in range(-75, 76, 5):
            self.gaze_plats.append(VecPos(15, i))
        for i in range(75, -76, -5):
            self.gaze_plats.append(VecPos(45, i))
        # for i in range(1, 31, 5):
        #     self.gaze_plats.append(VecPos(15 + i, 90 - 3 * i))
        # for i in range(1, 31, 5):
        #     self.gaze_plats.append(VecPos(45 - i, -3 * i))

    def execute(self):
        if self.bb.falled:
            self.bb.falled = False
            pos = len(self.gaze_plats) // 2
            self.iter = iter(self.gaze_plats)
            if self.bb.ball_field_last.y > 0:
                for _ in range(pos):
                    self.cur_plat = next(self.iter)

        ret = super(GoalKeeperScan, self).execute()
        return ret


class GoalKeeperStayHome(DynamicGuardSelector):

    def __init__(self, bb):
        children = [(GoalKeeperGoHome(bb), GotOutOfGoalKeeperHomePos(),
                     GotGoalKeeperHomePos()), (Do(bb), None, None)]
        super(GoalKeeperStayHome, self).__init__(children,
                                                 buffer_size=1,
                                                 use_exit=True)


class GoalKeeperGoHome(Walk, GoalKeeperBase):
    """Go to guard position."""

    def __init__(self, bb):
        self.get_attr()
        super(GoalKeeperGoHome, self).__init__(bb,
                                               walk_type='normal',
                                               dist_tol=80,
                                               angle_tol=30)
        self.x_tol = 20
        self.y_tol = 15

    def execute(self):
        if self.home_pos is None:
            home_x = self.bb.param.field_length / 2 - self.home_x
            if self.bb.param.attack_right:
                self.home_pos = VecPos(-home_x, self.home_y, 0)
            else:
                self.home_pos = VecPos(home_x, self.home_y, 180)
        # Don't move frequently if already got dest
        # if self.got_dest:
        #     self.dist_tol += 70
        #     self.angle_tol += 20
        #     self.y_tol += 40
        #     self.x_tol += 40
        if abs(self.bb.vision_info.robot_pos.y) < self.bb.param.goal_width / 2:
            self.walk_type = BodyCommand.WALK_POS

        if self._got_dest_simple(self.home_pos,
                                 dis_tol=self.dist_tol,
                                 angle_tol=self.angle_tol,
                                 x_tol=self.x_tol,
                                 y_tol=self.y_tol):
            self.crouch()
            return Status.SUCCEEDED
        else:
            self.bb.action_cmd.bodyCmd = BodyCommand(self.home_pos.x,
                                                     self.home_pos.y,
                                                     self.home_pos.z,
                                                     self.walk_type)
            self.bb.behavior_info.dest = self.home_pos
            self.bb.behavior_info.final_dest = self.home_pos
            # print("bodycommand changed\n\n")
            return Status.RUNNING


class GotGoalKeeperHomePos(ConditionLeaf, GoalKeeperBase):

    def __init__(self):
        super(GotGoalKeeperHomePos, self).__init__()
        self.get_attr()

    def condition(self):
        """
        Check if robot is near home line

        :return: whether robot is near home with hysteresis
        :rtype: bool
        """
        if self.home_pos is None:
            home_x = self.get_bb().param.field_length / 2 - self.home_x
            if self.get_bb().param.attack_right:
                self.home_pos = VecPos(-home_x, self.home_y, 0)
            else:
                self.home_pos = VecPos(home_x, self.home_y, 180)

        robot_pos = self.get_bb().vision_info.robot_pos

        # if self.get_bb().param.attack_right:
        #     robot_x_near = robot_pos.x > self.home_pos.x
        # else:
        #     robot_x_near = robot_pos.x < self.home_pos.x
        robot_dis_near = get_dis(self.home_pos, robot_pos) <= self.home_dis_max
        robot_angle_near = fabs(robot_pos.z -
                                self.home_pos.z) < self.home_angle_max

        # return robot_x_near and robot_dis_near and robot_angle_near
        return robot_dis_near and robot_angle_near


class GotOutOfGoalKeeperHomePos(ConditionLeaf, GoalKeeperBase):

    def __init__(self):
        super(GotOutOfGoalKeeperHomePos, self).__init__()
        self.get_attr()

    def condition(self):
        """
        Check if robot is near home line with hysteresis

        :return: whether robot is near home with hysteresis
        :rtype: bool
        """
        if self.home_pos is None:
            home_x = self.get_bb().param.field_length / 2 - self.home_x
            if self.get_bb().param.attack_right:
                self.home_pos = VecPos(-home_x, self.home_y, 0)
            else:
                self.home_pos = VecPos(home_x, self.home_y, 180)
        robot_pos = self.get_bb().vision_info.robot_pos

        robot_dis_near = get_dis(
            self.home_pos,
            robot_pos) <= self.home_dis_max + self.home_dis_max_hys
        angle = abs_angle_diff(robot_pos.z - self.home_pos.z)
        robot_angle_near = angle < self.home_angle_max + self.home_angle_max_hys

        return not (robot_dis_near and robot_angle_near)


class GoalKeeperAlign(GoalKeeperBase):

    def execute(self):
        if not self.bb.see_ball:
            return Status.FAILED

        p = self.get_align_point(self.kick_line_center)

        if self._got_dest_simple(p):
            self.crouch()
            return Status.SUCCEEDED
        else:
            self.goto_global(p)
            return Status.RUNNING


class KeepGoal(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            # TODO: Save ball if it's too fast
            # Go to attack if ball is in the attack zone
            (Attack(bb), GuardSelector(BallInGoalieAttackZone(), Kicking()),
             GuardSequence(BallOutOfGoalieAttackZone(), Invert(Kicking()))),
            # Align to ball
            (GoalKeeperAlign(bb), None, None)
            # GuardSelector(GotGoalHomePos(), BallInGoalieAttackZone()),
            # GotOutOfGoalHomePos())
        ]
        super(KeepGoal, self).__init__(children, buffer_size=3, use_exit=True)


class GoalKeeperBrain(Skill):
    '''
    Set attack target for goalie.
    '''

    def __init__(self, bb):
        super(GoalKeeperBrain, self).__init__(bb)
        self.defend_radius = 75
        self.default_attack_target = bb.attack_target
        self.magic_y = 30
        self.target = VecPos(0, 300)

    def execute(self):
        # set default target
        self.bb.attack_target = self.target

        # check ball
        if not self.bb.see_ball:
            return Status.RUNNING

        # ball position
        bally = self.bb.ball_global.y

        # enable kicking
        self.bb.kick_enabled = True

        # set target
        if bally > 75:
            self.target = VecPos(0, 400)
        elif bally < -75:
            self.target = VecPos(0, -400)
        self.bb.attack_target = self.target
        return Status.RUNNING


class KeepGoalLogic(Parallel):

    def __init__(self, bb):
        # args = (GoalKeeperBrain(bb), SimpleKeepGoal(bb))
        args = (
            GoalKeeperBrain(bb),
            DynamicGuardSelector([
                # See ball
                (Parallel(TrackBall(bb), KeepGoal(bb)), BallSeen()),
                # TODO: Handle TeamBallSeen()
                # Find ball if it's not in my own sight and go home
                (Parallel(FindBall(bb, look_down=True),
                          GoalKeeperGoHome(bb)), None)
            ]),
        )

        super(KeepGoalLogic, self).__init__(*args)


class SimpleKeepGoal(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (DynamicGuardSelector([
                (Attack(bb), GuardSelector(BallSeen(), TeamBallSeen(),
                                           Kicking())),
                (SeekBall(bb), None),
            ]), BallInGoalieAttackZone(), BallOutOfGoalieAttackZone()),
            (Parallel(
                DynamicGuardSelector([(TrackBall(bb), BallSeen()),
                                      (ScanFieldDown(bb), None)]),
                GoalKeeperGoHome(bb)), GotOutOfGoalHomePos(),
             GuardSelector(BallInGoalieAttackZone(), GotGoalHomePos())),
            (TrackBall(bb), BallSeen(), None),
            (ScanFieldDown(bb), None, None),
        ]
        super(SimpleKeepGoal, self).__init__(children,
                                             buffer_size=5,
                                             use_exit=True)


class SimpleGoalKeeper(Role):

    def __init__(self, bb):
        super(SimpleGoalKeeper, self).__init__(bb)
        root = DynamicGuardSelector([(Parallel(PrepareIMU(bb),
                                               InitialParticles(bb)),
                                      LowerBoardReconnected()),
                                     (Do(bb, cmd='look'), IMUInitialized()),
                                     (ScanField(bb), ReEntryLocalization()),
                                     (ScanFieldNew(bb), MotionReStable()),
                                     (SimpleKeepGoal(bb), None)])
        self.add_child(root)


class GoalieReadyToSave(ConditionLeaf, GoalKeeperBase):
    """
    Check whether goalie pos y in goal area(Whether able to save ball)
    """

    def __init__(self):
        super(GoalieReadyToSave, self).__init__()
        self.get_attr()

    def condition(self):
        if self.home_pos is None:
            home_x = self.get_bb().param.field_length / 2 - self.home_x
            if self.get_bb().param.attack_right:
                self.home_pos = VecPos(-home_x, self.home_y, 0)
            else:
                self.home_pos = VecPos(home_x, self.home_y, 180)
        robot_pos = self.get_bb().vision_info.robot_pos
        return abs(robot_pos.y) < self.get_bb(
        ).param.goal_width / 2 and abs_angle_diff(robot_pos.z -
                                                  self.home_pos.z) < 90


class BallOutOfGoalieAttackZone(ConditionLeaf):
    """
    Check whether to ignore ball cuz ball is invisible or too far
    """

    def __init__(self):
        super(BallOutOfGoalieAttackZone, self).__init__()
        # ignore ball if ball is out of this position [cm]
        self.ball_ignore_x = 150
        # hysteresis distance to ignore ball [cm]
        self.ball_ignore_x_hys = 70

    def condition(self):
        if not self.get_bb().see_ball:
            return False

        ball_global = self.get_bb().ball_global
        if self.get_bb().param.attack_right:
            ball_dis_near = ball_global.x < (
                -self.get_bb().param.field_length / 2 + self.ball_ignore_x +
                self.ball_ignore_x_hys)
        else:
            ball_dis_near = ball_global.x > (
                self.get_bb().param.field_length / 2 - self.ball_ignore_x -
                self.ball_ignore_x_hys)

        return not ball_dis_near


class BallInGoalieAttackZone(ConditionLeaf):

    def __init__(self):
        super(BallInGoalieAttackZone, self).__init__()
        # if ball in margin position regarding to goal area, then attack [cm]
        self.attack_margin = VecPos(50, 130)
        # attack margin hysteresis (value added to attack margin) [cm]
        self.attack_margin_hys = VecPos(25, 15)
        # Attack radius for defender
        self.attack_area = VecPos(100, 130)
        self.is_ball_near = False

    def ball_near_by(self):
        ball_field = self.get_bb().ball_field
        if not self.is_ball_near:
            if abs(ball_field.x) < self.attack_area.x and abs(
                    ball_field.y) < self.attack_area.y:
                self.is_ball_near = True
        else:
            if abs(ball_field.x
                  ) > self.attack_area.x + self.attack_margin_hys.x or abs(
                      ball_field.y
                  ) > self.attack_area.y + self.attack_margin_hys.y:
                self.is_ball_near = False

        return self.is_ball_near

    def ball_in_zone(self, xd, yd):
        """
        Check whether ball is in extended goal zone

        :param xd: extending position x
        :type xd: float
        :param yd: extending position y
        :type yd: float
        :return: whether ball is in desired zone
        :rtype: bool
        """
        if not self.get_bb().see_ball:
            return False

        ball_global = self.get_bb().ball_global
        if self.get_bb().param.attack_right:
            line_x = -self.get_bb().param.field_length / 2 + self.get_bb(
            ).param.goal_area_length + xd
            ball_in_zone_x = ball_global.x < line_x
        else:
            line_x = self.get_bb().param.field_length / 2 - self.get_bb(
            ).param.goal_area_length - xd
            ball_in_zone_x = ball_global.x > line_x

        line_y = self.get_bb().param.goal_area_width / 2 + yd
        ball_in_zone_y = fabs(ball_global.y) < line_y

        return ball_in_zone_x and ball_in_zone_y

    def condition(self):
        if not self.get_bb().see_ball:
            return False

        return self.ball_in_zone(self.attack_margin.x + 150,self.attack_margin.y + 75)##self.ball_near_by() or 
