from dbehavior.btree.branch import DynamicGuardSelector, Parallel, Sequence, GuardSelector, GuardSequence
from dbehavior.btree.leaf import (BallSeen, KickEnabled, AttackLeft,
                                  AttackRight, KickSuccess, MateBallHandling,
                                  TeamBallSeen, Kicking, Wait)
from dbehavior.skill import (Dribble, KickHead, TrackBall, SeekBall, GoKick,
                             FindBall, AssistBall, ScanFieldNew)

from dbehavior.btree.decorator import RepeatSeconds
from dbehavior.core import Do



class AttackBall(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            # If last motion complete, send a new command
            (KickHead(bb), Kicking()),
            (Parallel(GoKick(bb), TrackBall(bb)), None)
        ]
        super(AttackBall, self).__init__(children)

    def run(self):
        if not self.get_bb().kicking:
            self.get_bb().set_state('ball_handling')
            self.get_bb().announce_ball_handling()
        else:
            self.get_bb().set_state('playing')
        self.get_bb().assist_point = None
        super(AttackBall, self).run()


class AfterAttack(Parallel):

    def __init__(self, bb):
        args = (Do(bb, 'crouch'), FindBall(bb, look_down=True))
        super(AfterAttack, self).__init__(*args)

    def run(self):
        # Release ball handling after kicking ball
        self.get_bb().set_state('searching')
        super(AfterAttack, self).run()


class Attack(DynamicGuardSelector):

    def __init__(self, bb):
        children = [
            # TODO(daiz) Assist when both teammate and me are handing ball
            (AssistBall(bb), MateBallHandling()),
            (AfterAttack(bb), KickSuccess()),
            (AttackBall(bb), None)
        ]
        super(Attack, self).__init__(children)
