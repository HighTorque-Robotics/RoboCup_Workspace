from dbehavior.btree.branch import (Sequence, Parallel, GuardSequence,
                                    DynamicGuardSelector)
from dbehavior.btree.leaf import (LowerBoardReconnected, IMUInitialized,
                                  ReEntryOccurred, ReEntryLocalization,
                                  BallSeen, JoystickStopped)
from dbehavior.btree.decorator import Invert
from dbehavior.skill import PrepareIMU, Attack, ScanField, FindBall, InitialParticles, GoToInitPos
from dbehavior.core import Do, Role
from dbehavior.role.striker import Strike


class Fake(Role):

    def __init__(self, bb):
        super(Fake, self).__init__(bb)
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb),
                      InitialParticles(bb)), LowerBoardReconnected()),
            (Do(bb, cmd='look'), IMUInitialized()),
            (Parallel(Do(bb, 'crouch'), FindBall(bb)), JoystickStopped()),
            (ScanField(bb), ReEntryLocalization()),
            (Parallel(GoToInitPos(bb), FindBall(bb)),
             GuardSequence(ReEntryOccurred(), Invert(BallSeen()))),
            (Strike(bb), None)
        ])
        self.add_child(root)
