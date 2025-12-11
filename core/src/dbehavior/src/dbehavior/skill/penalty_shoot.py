from dbehavior.btree.core import Status
from dbehavior.btree.branch import Parallel, DynamicGuardSelector, GuardSelector
from dbehavior.btree.leaf import (GCPenaltyShootInitial, GCPenaltyShootSet,
                                  GCPenaltyShootPlaying)

from dbehavior.core import Skill
from dbehavior.util import VecPos
from dbehavior.skill import Attack, FindBall
from math import fabs
# from random import random
import random


class PenaltyBrain(Skill):
    """
    Determine attack target for penalty shoot.
    """

    def __init__(self, bb):
        super(PenaltyBrain, self).__init__(bb)
        self.magic_y = 100 # 70 to bypass enemy goalkeeper, used to be 30
        self.attack_goal_positive_part = True
        self.goal_set = False
        self.attack_goal_positive_part = random.randint(0,1)

    def check_enemy_goalie(self, obstacles):
        for obstacle in obstacles:
            if (self.bb.param.attack_right and 350 <= obstacle.x <= 550) or \
                    (not self.bb.param.attack_right and -350 <= obstacle.x <= -550):
                return obstacle
        return None

    def execute(self):
        # determine attack part by obstacle position
        # if self.bb.vision_info.see_obstacle:
        #     obstacles = self.bb.vision_info.obstacles_global
        #     enemy_goalie = self.check_enemy_goalie(obstacles)
        #     if enemy_goalie is not None:
        #         self.attack_goal_positive_part = enemy_goalie.y < 0
        #         self.goal_set = True
        
        # if not self.goal_set:
        #     if random() > 0.5:
        #         self.attack_goal_positive_part = True
        #     else:
        #         self.attack_goal_positive_part = False
            

        # the same as StrikerBrain
        x = self.bb.vision_info.robot_pos.x
        y = self.bb.vision_info.robot_pos.y

        self.bb.kick_enabled = True

        if self.bb.param.attack_right:
            # set target
            if self.attack_goal_positive_part:
                target = VecPos(550, self.magic_y)
            else:
                target = VecPos(550, -self.magic_y)

            if fabs(y) > 130:
                target.x -= 100

            
            # set enable kick
            if True: #450 - 100 > x > 450 - kick_ability:
                self.bb.kick_enabled = True
            # else:
            #     self.bb.kick_enabled = False   # FIXME: should we disable kick?

        else:
            if self.attack_goal_positive_part:
                target = VecPos(-550, self.magic_y)
            else:
                target = VecPos(-550, -self.magic_y)

            if fabs(y) > 130:
                target.x += 100

            if True: #-(450 - 100) < x < -(450 - kick_ability):
                self.bb.kick_enabled = True
            # else:
            #     self.bb.kick_enabled = False   # FIXME: should we disable kick?

        self.bb.attack_target = target

        return Status.RUNNING


class PenaltyAttack(Parallel):

    def __init__(self, bb):
        args = (
            PenaltyBrain(bb),
            Attack(bb)
        )
        super(PenaltyAttack, self).__init__(*args)


class PenaltyShoot(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (FindBall(bb), GuardSelector(
                GCPenaltyShootInitial(),
                GCPenaltyShootSet())),
            (PenaltyAttack(bb), GCPenaltyShootPlaying()),
        ]
        super(PenaltyShoot, self).__init__(children)
