from dbehavior.core import Role, Do, Skill
from dbehavior.btree.branch import (GuardSelector, GuardSequence,
                                    DynamicGuardSelector, Parallel)
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  GCLost, GCReEntry, GCInitialEntry,
                                  GCNormalPlaying, GCFinished, GCSet,
                                  GCFreeKick, GCPenaltyShoot, ResetPos,
                                  GCConnectionInited, GCNonKickOffPending,
                                  MotionReStable, BallSeen, TeamBallSeen, 
                                  GCKickOffSupporterDelay, GCNonKickOffPlaying,
                                  GCOurThrowIn, GCEnemyThrowIn)
from dbehavior.btree.decorator import Invert
from dbehavior.btree.core import Status
from dbehavior.skill import (PrepareIMU, FreeKick, PenaltyShoot,
                             InitialParticles, ReEntry, InitialEntry,
                             ScanField, ScanFieldNew, AssistBall, 
                             GoFuckPos, FindBall)

from dbehavior.role.striker import Strike
from dbehavior.role.defender import Defend, SimpleDefend
from dbehavior.role.goal_keeper import KeepGoalLogic, GoalKeeper

from functools import partial


class GCPlay(DynamicGuardSelector):

    def __init__(self, bb, playing_role=Strike):
        children = [
            (PenaltyShoot(bb), GCPenaltyShoot()),
            (FreeKick(bb), GCFreeKick()),
            (Parallel(
                Do(bb),
                ScanField(bb)
            ), GCFinished()),
            (InitialEntry(bb), GCInitialEntry()),
            (ReEntry(bb), GuardSequence(
                GCReEntry(),
                Invert(GuardSelector(BallSeen(), TeamBallSeen()))
            )),
            (GoFuckPos(bb), GCKickOffSupporterDelay()),
            # (FindBall(bb), GCNonKickOffPending()),
            (FindBall(bb), GCNonKickOffPlaying()),
            (playing_role(bb), GCNormalPlaying()),
            (Parallel(
                Do(bb),
                ScanField(bb)
            ), None)
        ]
        super(GCPlay, self).__init__(children)


class Game(Role):

    def __init__(self, bb, playing_role=Strike):
        super(Game, self).__init__(bb)
        self.add_child(DynamicGuardSelector([
            (Parallel(
                PrepareIMU(bb),
                # DynamicGuardSelector([(InitialParticles(bb), ResetPos())]),
                InitialParticles(bb)
            ), LowerBoardReconnected()),
            # (Do(bb, cmd='look'), IMUInitialized()),
            # (Parallel(
            #     Do(bb),
            #     ScanField(bb)
            # ), Invert(GCConnectionInited())),
            (ScanFieldNew(bb), MotionReStable()),
            (GCPlay(bb, playing_role), Invert(GCLost())),
            (playing_role(bb), None)
        ]))


GCStriker = partial(Game, playing_role=Strike)
GCSupporter = partial(Game, playing_role=Strike)
GCGoalKeeper = partial(Game, playing_role=GoalKeeper)
GCDefender = partial(Game, playing_role=SimpleDefend)


class GCMiss(Skill):

    def __init__(self, bb):
        super(GCMiss, self).__init__(bb)
        self.total = 0
        self.lost = 0

    def on_start(self):
        super(GCMiss, self).on_start()
        self.total = 0
        self.lost = 0

    def execute(self):
        # if not self.bb.gc_info.connected:
        if self.bb.gc_lost:
            self.lost += 1
        self.total += 1

        print('Total: {}\tMiss: {} ({}%)'.format(self.total, self.lost, self.lost * 100. / self.total))
        return Status.RUNNING


class GCCheck(Role):

    def __init__(self, bb):
        super(GCCheck, self).__init__(bb)
        self.add_child(GCMiss(bb))
