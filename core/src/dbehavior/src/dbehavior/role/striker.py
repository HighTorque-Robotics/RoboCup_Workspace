from dbehavior.core import Skill, Role, Do
from dbehavior.skill import (PrepareIMU, Attack, ScanField, ScanFieldNew,
                             FindBall, SeekBall, SearchBall, InitialParticles,
                             GoToInitPos, ScanFieldSlow,
                             FindLastBall, EntryScanField)
from dbehavior.btree.core import Status
from dbehavior.btree.branch import (Parallel, DynamicGuardSelector,
                                    GuardSequence, GuardSelector, Sequence)
from dbehavior.btree.decorator import Invert
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  ReEntryOccurred, ReEntryLocalization,
                                  BallSeen, TeamBallSeen, MotionReStable,
                                  BallLostRecent, Kicking, Wait, ReEntrying)
from dbehavior.util import VecPos, Timer
from dbehavior.util.mathutil import degree_between, get_magnitude
from dmsgs.msg import TeamInfo
from math import fabs
import random


class StrikerBrain(Skill):
    """
    Determine attack target.
    """

    def __init__(self, bb, goal_time_out=30):
        super(StrikerBrain, self).__init__(bb)
        self.magic_y = 30
        if bb.param.attack_right:
            self.target = VecPos(550, self.magic_y)
        else:
            self.target = VecPos(-550, self.magic_y)
        bb.attack_target = self.target
        self.goals_mid = []
        self.num_to_determine = 5
        self.goal_timer = Timer()
        self.goal_time_out = goal_time_out
        self.use_goal_target = False
        self.left_coef = 1
        self.right_coef = 1
        self.random_side = (random.randint(0,1)-0.5)*2

    def execute(self):
        x = self.bb.vision_info.robot_pos.x
        y = self.bb.vision_info.robot_pos.y
        ballx = self.bb.vision_info.ball_global.x
        bally = self.bb.vision_info.ball_global.y

        if self.goal_timer.elapsed() > self.goal_time_out:
            self.use_goal_target = False
            self.goals_mid = []
            self.goal_timer.restart()
        my_goal = None
        if self.bb.vision_info.see_goal:
            if self.bb.param.attack_right:
                my_goal = self.bb.vision_info.goal_global.x < 0
            else:
                my_goal = self.bb.vision_info.goal_global.x > 0

        # if self.bb.vision_info.see_goal and not my_goal:
        #     goal = VecPos.from_vector3(self.bb.vision_info.goal_global)
        #     self.goals_mid.append(goal)
        #     if len(self.goals_mid) > self.num_to_determine:
        #         self.target = VecPos()
        #         for pt in self.goals_mid:
        #             self.target.x += pt.x
        #             self.target.y += pt.y
        #         self.target.x /= len(self.goals_mid)
        #         self.target.y /= len(self.goals_mid)
        #         self.bb.attack_target = self.target
        #         # Shoot into the goal deeper
        #         self.target.x += self.target.x / abs(self.target.x) * 100
        #         self.goals_mid = []
        #         self.use_goal_target = True
        #         self.goal_timer.restart()
        # elif not self.use_goal_target:
        
        if self.bb.target_used:
            
            # set target.x
            if fabs(ballx) < 250 or fabs(bally) < 100:
                if self.bb.param.attack_right:
                    self.target.x = 550
                else:
                    self.target.x = -550
            elif fabs(ballx) > 300 and fabs(bally) > 150:
                if self.bb.param.attack_right:
                    self.target.x = 450
                else:
                    self.target.x = -450
            else:
                if fabs(x) > fabs(self.target.x):
                    self.target.x += self.target.x / abs(self.target.x) * 70

            # set target.y
            if fabs(ballx) > 300 and fabs(bally) > 100:
                # print("target used: set y to magic_y")
                # self.target.y = -bally / abs(bally) * self.magic_y
                self.target.y = bally / abs(bally) * self.magic_y
            elif fabs(ballx) < 250:
                if bally > 70:
                    self.target.y = self.magic_y
                elif bally < -70:
                    self.target.y = -self.magic_y

                # if fabs(bally) > self.bb.param.goal_width / 2:
                #     self.target.x -= 100
                # elif fabs(ballx) > 300:
                #     pass
                # if fabs(bally) > self.bb.param.goal_width / 2:
                #     self.target.x += 100
            # if fabs(ballx) > 300:

            # if fabs(ballx) > 200 and fabs(y) < 70 and fabs(self.target.y) == 30:
            #     self.target.y *= 2
            self.bb.target_used = False

        self.bb.target_used = True
        self.bb.attack_target = self.target

        # enable kicking
        self.bb.kick_enabled = True

        # Don't kick straight when kickoff
        if self.bb.gc_info.kickoff and self.bb.timer_gc_playing.elapsed() < 15:
            if self.bb.param.attack_right:
                self.bb.attack_target = VecPos(550, 320 * self.random_side)
            else:
                self.bb.attack_target = VecPos(-550, 320 * self.random_side)

        return Status.RUNNING


class Strike(Parallel):

    def __init__(self, bb):
        args = (StrikerBrain(bb),
                DynamicGuardSelector([
                    (Attack(bb),
                     GuardSelector(BallSeen(), TeamBallSeen(), Kicking())),
                    # (FindLastBall(bb), BallLostRecent()),
                    # (Parallel(GoToInitPos(bb), FindBall(bb)),
                    #  GuardSequence(ReEntrying(), Invert(BallSeen()))),
                    (SeekBall(bb), None),
                ]))
        super(Strike, self).__init__(*args)


class Striker(Role):
    """
    A striker is actively pursuing the ball.
    """

    def __init__(self, bb):
        super(Striker, self).__init__(bb)
        root = DynamicGuardSelector([(Parallel(PrepareIMU(bb),
                                               InitialParticles(bb)),
                                      LowerBoardReconnected()),
                                     (Sequence(
                                         Wait(1),
                                         EntryScanField(bb),
                                     ), IMUInitialized()),
                                    #  (Parallel(GoToInitPos(bb), FindBall(bb)),
                                    #   BallSeen()),
                                    #  (ScanFieldNew(bb), MotionReStable()),
                                     (Strike(bb), None)])

        self.add_child(root)


class FakeStriker(Role):
    """
    Fake GCStriker pretend to be listening to GC
    """

    def __init__(self, bb):
        super(FakeStriker, self).__init__(bb)
        root = DynamicGuardSelector([(Parallel(PrepareIMU(bb),
                                               InitialParticles(bb)),
                                      LowerBoardReconnected()),
                                     (Do(bb, cmd='look'), IMUInitialized()),
                                     (EntryScanField(bb),
                                      GuardSelector(IMUInitialized(),
                                                    ReEntryLocalization())),
                                     (Parallel(GoToInitPos(bb), FindBall(bb)),
                                      GuardSequence(ReEntryOccurred(),
                                                    Invert(BallSeen()))),
                                     (ScanFieldNew(bb), MotionReStable()),
                                     (Strike(bb), None)])

        self.add_child(root)
