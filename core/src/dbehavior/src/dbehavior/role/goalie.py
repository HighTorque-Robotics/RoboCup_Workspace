from dbehavior.core import Role
from dbehavior.skill import Attack, ScanField, PrepareIMU, GoalieBodyLineUp, GoalieDefend, TrackBall, InitialGoaliePos
from dbehavior.btree.branch import DynamicGuardSelector, GuardSelector, Parallel
from dbehavior.btree.leaf import BallInOurPenaltyArea, CloseToBall, LowerBoardReconnected, BallSeen
from dbehavior.btree.decorator import Invert
from dbehavior.util import VecPos


class GoalieLineUp(Parallel):
    def __init__(self, bb):
        args = (GoalieBodyLineUp(bb), TrackBall(bb))
        super(GoalieLineUp, self).__init__(*args)


class GoalieScan(ScanField):
    def __init__(self, bb):
        super(GoalieScan, self).__init__(bb)
        self.gaze_plats = [VecPos(15, 60), VecPos(35, 0), VecPos(15, -60)]


class Goalie(Role):
    """
    A goalie is positioned inside the penalty box, and the last line
    of defense. It may touch the ball, and it has special protection.
    """

    def __init__(self, bb):
        super(Goalie, self).__init__(bb)

        root = DynamicGuardSelector([
            (Parallel(PrepareIMU(bb), InitialGoaliePos(bb)),
             LowerBoardReconnected()),
            # (Attack(bb), GuardSelector(BallInOurPenaltyArea(), CloseToBall())),
            # (FindBall(bb), None)
            # (GoalieLineUp(bb), BallSeen()),
            (GoalieDefend(bb), BallInOurPenaltyArea()),
            (TrackBall(bb), BallSeen()),
            (GoalieScan(bb), Invert(BallSeen()))
        ])

        self.add_child(root)
