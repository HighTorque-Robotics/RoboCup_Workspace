from dbehavior.core import Role, Do
from dbehavior.skill.penalty_shoot import PenaltyAttack, PenaltyShoot
from dbehavior.btree.branch import Parallel, DynamicGuardSelector
from dbehavior.skill import (PrepareIMU, FindBall, InitialParticles)
from dbehavior.btree.leaf import (LowerBoardReconnected, BallSeen,
                                  IMUInitialized, GCPenaltyShoot)
from dbehavior.btree.decorator import Invert

class PenaltyKicker(Role):
    def __init__(self, bb):
        super(PenaltyKicker, self).__init__(bb)
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb), InitialParticles(bb)), LowerBoardReconnected()),
            (Do(bb, cmd='look'), IMUInitialized()),
            (FindBall(bb, look_down=True), Invert(BallSeen())),
            (PenaltyShoot(bb), GCPenaltyShoot()),
            # (PenaltyAttack(bb), GCPenaltyShoot()),
            # (Parallel(FindBall(bb), Do(bb)), None)
            # (PenaltyAttack(bb), None)
        ])
        self.add_child(root)
