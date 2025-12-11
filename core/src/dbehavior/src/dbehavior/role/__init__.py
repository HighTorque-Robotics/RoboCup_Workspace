from .dummy import Doll, SmartDoll, DemoMain, Dummy, GetImage
from .striker import Striker, Strike, FakeStriker
from .goal_keeper import GoalKeeper, SimpleGoalKeeper
from .penalty_kicker import PenaltyKicker
from .defender import Defender, Defend
from .game import (Game, GCCheck, GCSupporter,
                   GCStriker, GCGoalKeeper, GCDefender)
from .fake import Fake
from .foo import (Foo, TestEuler, TestLocalization,
                  TestKick, TestAttack,
                  TestAttackBall, TestGoToPos,
                  TestPatrol, TestSearchBall,
                  TestTrackCircle, TestTrackGoal,
                  TestGoBack, TestKeepBall)

__all__ = (
    Doll, SmartDoll, DemoMain, Dummy, GetImage,
    TestKick, TestAttack, TestPatrol, FakeStriker,
    TestSearchBall, TestLocalization, TestTrackCircle,
    Striker, Defender, GoalKeeper,
    SimpleGoalKeeper, PenaltyKicker,
    Game, GCCheck, GCStriker, GCGoalKeeper, GCDefender,
    Foo, TestEuler, Fake, TestTrackGoal, GCSupporter,
    TestGoBack, TestGoToPos, TestKeepBall
)
