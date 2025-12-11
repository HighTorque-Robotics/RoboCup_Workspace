from dbehavior.btree.core import Status
from dbehavior.btree.branch import DynamicGuardSelector, Sequence, Parallel
from dbehavior.btree.leaf import (GCState2Freeze, GCState2Ready,
                                  GCOurFreeKick, GCEnemyFreeKick,
                                  GCOurThrowIn, GCEnemyThrowIn,
                                  GCThrowInWait, GCThrowInPlacing, IsGoalie,
                                  CloseToBall, Wait, FarFromBall, GCBallNotInPlay)
from dbehavior.skill import FindBall, GoKick
from dbehavior.core import Skill, Do
from dbehavior.util import VecPos, Timer
from dbehavior.util.mathutil import get_magnitude


class GoAwayFromBall(Skill):

    def __init__(self, bb, dis, tol=10):
        super(GoAwayFromBall, self).__init__(bb)
        self.dis = dis
        self.tol = tol

    def execute(self):
        if not self.bb.see_ball:
            return Status.FAILED
        ball_field = VecPos.from_vector3(self.bb.ball_field)
        if ball_field is None:
            return Status.FAILED

        if get_magnitude(ball_field) <= self.dis + self.tol:
            self.backward()
            return Status.RUNNING
        else:
            self.crouch()
            return Status.SUCCEEDED


class KeepDistanceFromBall(DynamicGuardSelector):

    def __init__(self, bb, dis=80, tol=10):
        children = [
            (Parallel(FindBall(bb), GoAwayFromBall(bb, dis)),
             CloseToBall(dis-tol), FarFromBall(dis-tol+5)),
            (Parallel(FindBall(bb), GoKick(bb)),
             FarFromBall(dis+tol), CloseToBall(dis+tol-5)),
            (Parallel(FindBall(bb), Do(bb, 'crouch')), None, None)
        ]
        super(KeepDistanceFromBall, self).__init__(children, use_exit=True)


class SetEnemyFreeKick(Skill):

    def __init__(self, bb, flag=True):
        super(SetEnemyFreeKick, self).__init__(bb)
        self.flag = flag

    def execute(self):
        self.bb.enemy_free_kick = self.flag
        return Status.SUCCEEDED


class OurFreeKick(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (Parallel(FindBall(bb), Do(bb, 'crouch')), GCState2Freeze()),
            (KeepDistanceFromBall(bb, dis=40), GCState2Ready())
        ]
        super(OurFreeKick, self).__init__(children)


class EnemyFreeKick(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (Parallel(
                FindBall(bb),
                Do(bb, 'crouch'),
            ), GCState2Freeze()),
            (Sequence(
                KeepDistanceFromBall(bb, dis=75),
                SetEnemyFreeKick(bb, True)
            ), GCState2Ready()),
        ]
        super(EnemyFreeKick, self).__init__(children)


class OurThrowIn(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (Parallel(FindBall(bb), Do(bb, 'crouch')), GCThrowInWait()),
            (KeepDistanceFromBall(bb, dis=50), GCThrowInPlacing())
        ]
        super(OurThrowIn, self).__init__(children)


class EnemyThrowIn(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            (Parallel(FindBall(bb), Do(bb, 'crouch')), GCThrowInWait()),
            (GoAwayFromBall(bb, dis=80), GCThrowInPlacing())
        ]
        super(EnemyThrowIn, self).__init__(children)


class FreeKick(DynamicGuardSelector):
    """
    Excute free kick behaiviors according to the free kick side.
    Keep crouch when the ball is not in play
    TODO(daiz): Wait for 10s now, try to figure out whether the ball is moved by enemy
    """

    def __init__(self, bb):
        children = [
            # (OurThrowIn(bb), GCOurThrowIn()),
            # (EnemyThrowIn(bb), GCEnemyThrowIn()),
            (Parallel(Do(bb), FindBall(bb)), IsGoalie()),
            (OurFreeKick(bb), GCOurFreeKick()),
            (EnemyFreeKick(bb), GCEnemyFreeKick()),
            (Parallel(FindBall(bb), Do(bb, 'crouch')), GCBallNotInPlay()),
            (SetEnemyFreeKick(bb, False), None)
        ]
        super(FreeKick, self).__init__(children)
