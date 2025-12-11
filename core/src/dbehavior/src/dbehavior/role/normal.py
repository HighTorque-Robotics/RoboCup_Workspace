from dbehavior.core import Role, Do
from dbehavior.skill import (PrepareIMU, Attack, ScanField, ScanFieldNew,
                             FindBall, SeekBall, SearchBall, InitialParticles,
                             GoToInitPos)
from dbehavior.btree.branch import (Parallel, DynamicGuardSelector,
                                    GuardSequence, GuardSelector)
from dbehavior.btree.decorator import Invert
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  ReEntryOccurred, ReEntryLocalization,
                                  BallSeen, TeamBallSeen, MotionReStable)

from dbehavior.role.striker import Strike
from dbehavior.role.defender import Defend
from dbehavior.role.goal_keeper import KeepGoal

from functools import partial


class Normal(Role):

    def __init__(self, bb, playing_role):
        super(Normal, self).__init__(bb)
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb),
                      InitialParticles(bb)), LowerBoardReconnected()),
            # (Do(bb, cmd='look'), IMUInitialized()),
            (ScanField(bb),
             GuardSelector(IMUInitialized(), ReEntryLocalization())),
            (Parallel(GoToInitPos(bb), FindBall(bb)),
             GuardSequence(ReEntryOccurred(), Invert(BallSeen()))),
            (ScanFieldNew(bb), MotionReStable()),
            (playing_role(bb), None)
        ])

        self.add_child(root)


Striker = partial(Normal, playing_role=Strike)
GoalKeeper = partial(Normal, playing_role=KeepGoal)
Defender = partial(Normal, playing_role=Defend)
