from dbehavior.core import Skill, Do
from dbehavior.btree.core import Status
from dbehavior.btree.leaf import CircleSeen
from dbehavior.btree.decorator import RepeatSeconds, Invert
from dbehavior.btree.branch import Sequence, DynamicGuardSelector, Parallel
from dbehavior.util import Timer
from dbehavior.skill import TrackCircle, ScanField, TurnAround, ScanFieldNew


class Patrol(Skill):

    def __init__(self, bb):
        super(Patrol, self).__init__(bb)
        self.timer = Timer(left_shift=-9999)
        self.patrol_target = [bb.patrol_pos_a,
                              bb.patrol_pos_b]
        self.target_index = 0

    def on_start(self):
        super(Patrol, self).on_start()
        self.timer.restart()

    def execute(self):
        if self._got_dest_simple(self.patrol_target[self.target_index],
                                 dis_tol=50,
                                 angle_tol=20) and \
                self.timer.elapsed() > 3:
            self.target_index = (self.target_index + 1) % 2
            return Status.SUCCEEDED
        self.goto_global(self.patrol_target[self.target_index])
        # feed final and current destination to BehaviorInfo
        # print("Patrol: {}".format(self.patrol_target[self.target_index]))
        self.bb.behavior_info.dest = self.patrol_target[self.target_index]
        self.bb.behavior_info.final_dest = self.patrol_target[
            self.target_index]
        return Status.RUNNING


class SearchBall(Parallel):

    def __init__(self, bb):
        if bb.param.attack_right:
            self.turn_angle = -60
        else:
            self.turn_angle = 60
        args = (
            DynamicGuardSelector([
                (TrackCircle(bb), CircleSeen()),
                (ScanFieldNew(bb), None)
            ]),
            Sequence(
                RepeatSeconds(Do(bb), timeout=2.5),
                TurnAround(bb, look_down=False, target_angle=self.turn_angle),
                RepeatSeconds(Do(bb), timeout=2.5),
                Patrol(bb)
            )
        )
        super(SearchBall, self).__init__(*args)

    def run(self):
        self.get_bb().set_state('searching')
        super(SearchBall, self).run()
