from dbehavior.core import Skill
from dbehavior.btree.core import Status
from dbehavior.btree.decorator import Condition
from dbehavior.util import Timer, VecPos
from dbehavior.util.mathutil import degree_between, get_magnitude, deg2rad
from math import fabs, sin, cos
from dmsgs.msg import TeamInfo, BodyCommand
import rospy
from random import random


# Used for testing attack
class KickBrain(Skill):
    """
    Determine attack target.
    """

    def __init__(self, bb):
        super(KickBrain, self).__init__(bb)
        self.goals_mid = []
        self.num_to_determine = 10

    def execute(self):
        self.bb.kick_enabled = True
        # if len(self.bb.vision_info.goals_global) == 2:
        #     left_goalpost = self.bb.vision_info.goals_global[0]
        #     right_goalpost = self.bb.vision_info.goals_global[1]
        #     mid_goalpost = VecPos((left_goalpost.x + right_goalpost.x) / 2.0,
        #                           (left_goalpost.y + right_goalpost.y) / 2.0)
        #     self.goals_mid.append(mid_goalpost)
        #     if len(self.goals_mid) > self.num_to_determine:
        #         target = VecPos()
        #         for pt in self.goals_mid:
        #             target.x += pt.x
        #             target.y += pt.y
        #         target.x /= len(self.goals_mid)
        #         target.y /= len(self.goals_mid)
        #         self.bb.attack_target = target
        #         self.goals_mid = []
        #         print('goal attack target: {}'.format(target))
        if self.bb.ball_global is not None and \
                self.bb.behavior_info.team_play_state is TeamInfo.PLACING:
            ball_field = VecPos.from_vector3(self.bb.ball_field)
            if ball_field is not None and get_magnitude(ball_field) > 30:
                ball_global = VecPos.from_vector3(self.bb.ball_global)
                robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
                r2b = ball_global - robot_pos
                target = ball_global + r2b.normalize(75)
                target.z = degree_between(ball_global, robot_pos)
                self.bb.attack_target = target
        else:
            if self.bb.param.attack_right:
                self.bb.attack_target = VecPos(550, 0)
            else:
                self.bb.attack_target = VecPos(-550, 0)

        # print('attack_target: ({}, {})'.format(self.bb.attack_target.x,
        #                                        self.bb.attack_target.y))
        return Status.RUNNING


class EnableKick(Condition):
    """
    Check whether kicking is enabled.
    """

    def condition(self):
        return self.get_bb().kick_enabled


class KickHead(Skill):
    """
    Head motion when kicking.
    """

    def __init__(self, bb):
        super(KickHead, self).__init__(bb)
        self.kick_timer = Timer()

    def on_start(self):
        self.kick_timer = Timer()
        self.get_bb().target_used = True

    def on_end(self):
        super(KickHead, self).on_end()

    def execute(self):
        self.look_at(15, 0)
        self.bb.set_state('kicking')

        return Status.RUNNING


class GoKick(Skill):
    """
    Go to the ball and kick.
    """

    def __init__(self, bb):
        super(GoKick, self).__init__(bb)
        self.kick_timer = Timer()

    def on_start(self):
        self.kick_timer = Timer()
        self.bb.kicking = False
        self.get_bb().target_used = True

    def on_end(self):
        super(GoKick, self).on_end()

    def execute(self):
        ball_global = VecPos.from_vector3(self.bb.vision_info.ball_global)
        # Kick direction(global) of ball
        target = (self.bb.attack_target - ball_global).slope()
        enable_dribble = False
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
        if self.bb.param.pos_role == 'striker':
            if self.bb.param.attack_right:
                enable_dribble = ball_global.x < -150
            else:
                enable_dribble = ball_global.x >= 150
        else: 
            enable_dribble = False
        # here
        self.bb.action_cmd.bodyCmd = self.action_generator.dribble(ball_global.x, ball_global.y, target)

        # if enable_dribble:#self.bb.param.use_dribble:
        #     self.bb.action_cmd.bodyCmd = self.action_generator.dribble(ball_global.x, ball_global.y, target)
        # else:
        #     self.bb.action_cmd.bodyCmd = self.action_generator.kick_ball(ball_global.x, ball_global.y, target)

        return Status.RUNNING


class KickSide(Skill):
    """
    Kick the ball to the left or right.(Pass to team-mate)
    """

    def __init__(self, bb):
        super(KickSide, self).__init__(bb)

    def on_start(self):
        self.kick_timer = Timer()
        self.bb.kicking = False
        self.get_bb().target_used = True
        # positive for kick left, negtive for kick right
        self.kick_direction = None

    def on_end(self):
        super(KickSide, self).on_end()

    def execute(self):
        if self.kick_direction:
            closest_x = 300
            closest_y = None
            my_pos = self.get_bb().vision_info.robot_pos
            for i in range(1, self.get_bb().param.num_player + 1):
                info = self.get_bb().get_team_info(i)
                if info is not None:
                    dist_x = info.robot_pos.x - my_pos.x
                    if abs(dist_x) < abs(closest_x):
                        closest_x = dist_x
                        closest_y = info.robot_pos.y - my_pos.y
            if closest_y is None:
                self.kick_direction = (random() - .5)
            else:
                theta = -deg2rad(my_pos.z)
                relative_y = -closest_x * sin(theta) + closest_y * cos(theta)
                self.kick_direction = float(relative_y > 0) - .5

        # FIXME: kick side not impleted in dancer-motion.
        # self.bb.action_cmd.bodyCmd = self.action_generator.kick_ball(
        #     ball_global.x, ball_global.y, target)

        return Status.RUNNING