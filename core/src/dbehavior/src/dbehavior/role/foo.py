from dbehavior.role import Dummy
from dbehavior.skill import (GoKick, Euler, PrepareIMU, ScanField, SeekBall,
                             TrackBall, FindBall, TurnTo, SearchBall,
                             ScanFieldNew, AttackBall, Attack, Patrol,
                             TrackCircle, TrackGoal, GoToInitPos, GoToPos,
                             InitialParticles, FindLastBall, GoAwayFromBall,
                             SetOrientation)
from dbehavior.btree.leaf import (LowerBoardReconnected, BallSeen,
                                  BallLostRecent, IMUInitialized, CircleSeen,
                                  GoalSeen, Kicking)
from dbehavior.btree.branch import Sequence, Parallel, DynamicGuardSelector, GuardSelector
from dbehavior.skill.kick import KickBrain
from dbehavior.core import Do
from dbehavior.util import VecPos
from dbehavior.role.goal_keeper import SaveBall, BallInDanger, GoalKeeperScan, BallInGoalieAttackZone


class Foo(Dummy):
    """
    Foo role for testing.
    """

    def __init__(self, bb, root=None):
        super(Foo, self).__init__(bb)

        if root is not None:
            self.add_child(root)


# yapf: disable
class TestKeepBall(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            (Parallel(Do(bb), InitialParticles(bb)), LowerBoardReconnected()),
            (DynamicGuardSelector([
                (SaveBall(bb), BallInDanger()),
                (Parallel(
                    TrackBall(bb, speed=1)), None)
            ]), BallSeen()),
            (Parallel(
                GoalKeeperScan(bb)), None)
        ])
        super(TestKeepBall, self).__init__(bb, root)

# class TestKeepBall(Foo):

#     def __init__(self, bb):
#         root = DynamicGuardSelector([
#             (Parallel(Do(bb), InitialParticles(bb)), LowerBoardReconnected()),
#             (DynamicGuardSelector([
#                 (SaveBall(bb), BallInDanger()),
#                 (Attack(bb), BallInGoalieAttackZone()),
#                 (Parallel(
#                     TrackBall(bb, speed=0.1),
#                     GoToGuardPos(bb)), None)
#             ]), BallSeen()),
#             (Parallel(
#                 GoalKeeperScan(bb),
#                 GoToGuardPos(bb)), None)
#         ])
#         super(TestKeepBall, self).__init__(bb, root)
# yapf: enable


class TestGoBack(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            (Parallel(Do(bb), InitialParticles(bb)), LowerBoardReconnected()),
            (Parallel(TrackBall(bb), GoAwayFromBall(bb, dis=80,
                                                    tol=15)), BallSeen()),
            (FindBall(bb), None)
        ])
        super(TestGoBack, self).__init__(bb, root)


class TestGoToPos(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            # Reset to (-240, 0, 0) now
            (Parallel(Do(bb), InitialParticles(bb)), LowerBoardReconnected()),
            (GoToPos(bb, VecPos(-350, 0, 180)), None)
        ])
        super(TestGoToPos, self).__init__(bb, root)


class TestLocalization(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb),
                      SetOrientation(bb)), LowerBoardReconnected()),
            (Parallel(
                Do(bb),
                DynamicGuardSelector([(TrackCircle(bb), CircleSeen()),
                                      (TrackGoal(bb), GoalSeen()),
                                      (ScanFieldNew(bb), None)])), None)
        ])
        super(TestLocalization, self).__init__(bb, root)


class TestGoBack(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            (Parallel(Do(bb), InitialParticles(bb)), LowerBoardReconnected()),
            (Parallel(TrackBall(bb), GoAwayFromBall(bb, dis=80,
                                                    tol=15)), BallSeen()),
            (FindBall(bb), None)
        ])
        super(TestGoBack, self).__init__(bb, root)


class TestTrackCircle(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([(TrackCircle(bb), CircleSeen()),
                                     (ScanFieldNew(bb), None)])
        super(TestTrackCircle, self).__init__(bb, root)


class TestTrackGoal(Foo):

    def __init__(self, bb):
        root = Parallel(
            DynamicGuardSelector([(TrackGoal(bb), GoalSeen()),
                                  (ScanFieldNew(bb), None)]), Do(bb, 'crouch'))
        super(TestTrackGoal, self).__init__(bb, root)


class TestEuler(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([(PrepareIMU(bb), LowerBoardReconnected()),
                                     (Parallel(Euler(bb), FindBall(bb)), None)])
        super(TestEuler, self).__init__(bb, root)


class TestKick(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([(Parallel(PrepareIMU(bb),
                                               InitialParticles(bb)),
                                      LowerBoardReconnected()),
                                     (Parallel(GoKick(bb),
                                               TrackBall(bb)), BallSeen()),
                                     (ScanField(bb), None)])
        super(TestKick, self).__init__(bb, root)


class TestAttackBall(Foo):

    def __init__(self, bb):
        root = Parallel(
            KickBrain(bb),
            DynamicGuardSelector([
                (Parallel(Do(bb),
                          InitialParticles(bb)), LowerBoardReconnected()),
                (AttackBall(bb), GuardSelector(BallSeen(), Kicking())),
                (FindLastBall(bb), BallLostRecent()),
                (SeekBall(bb), None),
            ]))
        super(TestAttackBall, self).__init__(bb, root)


class TestAttack(Foo):

    def __init__(self, bb):
        root = Parallel(
            KickBrain(bb),
            DynamicGuardSelector([
                (Attack(bb), BallSeen()),
                (SeekBall(bb), None),
            ]))
        super(TestAttack, self).__init__(bb, root)


class TestPatrol(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([(PrepareIMU(bb), LowerBoardReconnected()),
                                     (Parallel(Patrol(bb),
                                               ScanField(bb)), None)])
        super(TestPatrol, self).__init__(bb, root)


class TestSearchBall(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([
            (PrepareIMU(bb), LowerBoardReconnected()),
            # (TrackBall(bb), BallSeen()),
            (SearchBall(bb), None)
        ])
        super(TestSearchBall, self).__init__(bb, root)


class TestTurnTo(Foo):

    def __init__(self, bb):
        root = DynamicGuardSelector([(PrepareIMU(bb), LowerBoardReconnected()),
                                     (TurnTo(bb, 180), None)])
        super(TestTurnTo, self).__init__(bb, root)


class TestPenaltyKick(Foo):

    def __init__(self, bb):
        # root = DynamicGuardSelector([
        #     (PenaltyAttack(bb), None),
        # ])
        # bb.param.role == 'PenaltyKicker'
        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb),
                      InitialParticles(bb)), LowerBoardReconnected()),
            (Do(bb, cmd='look'), IMUInitialized()),
            (FindBall(bb, look_down=True), Invert(BallSeen())),
            (PenaltyAttack(bb), None)
        ])
        # Sequence(
        #GoToPos(bb, VecPos(300, 300, 180)),
        # TurnTo(bb, 0),
        # SeekBall(bb)
        #)
        super(TestPenaltyKick, self).__init__(bb, root)
