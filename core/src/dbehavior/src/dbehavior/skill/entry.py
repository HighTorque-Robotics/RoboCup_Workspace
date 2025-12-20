from dbehavior.core import Skill, Do
from dbehavior.btree.core import Status
from dbehavior.btree.branch import (Parallel, DynamicGuardSelector,
                                    GuardSelector, GuardSequence)
from dbehavior.btree.leaf import (ReEntryLocalization, ReEntryOccurred,
                                  FarFromInitPos, BallSeen, CircleSeen,
                                  GoalSeen, GCNormalInitial, GCNormalReady,
                                  GCNormalSet, GCPenalised, GotOutOfInitPos,
                                  GotInitPos)
from dbehavior.btree.decorator import Invert
from dbehavior.util import VecPos
from dbehavior.skill import (FindBall, ScanField, ScanFieldNew,
                             InitialParticles, TrackCircle, TrackGoal, TurnTo,
                             TurnAround, EntryScanField)
from dmsgs.msg import BodyCommand


class Walk(Skill):
    """Common settings for walking skills."""

    def __init__(self, bb, walk_type='normal', dist_tol=30, angle_tol=15):
        """
        Initialization

        :param bb: blackboard
        :param walk_type: Type of walking
        """
        super(Walk, self).__init__(bb)
        if walk_type == 'translation':
            self.walk_type = BodyCommand.WALK_POS
        elif walk_type == 'normal':
            self.walk_type = BodyCommand.WALK_POS
        else:
            print('==================WARNNING=================\n',
                  'INVALID WALK TYPE! Using default type.')
            self.walk_type = BodyCommand.WALK_POS
        self.dist_tol = 10
        self.angle_tol = 10
        self.got_dest = False


class GoToInitPos(Walk):
    """Go to default position of role."""

    def execute(self):
        # if self.bb.lower_board_reconnected:
        #     self.crouch()
        #     return Status.RUNNING

        init_pos = self.bb.init_pos

        # Don't move frequently if already got dest
        if self.got_dest:
            self.dist_tol += 20
            self.angle_tol += 30

        if self._got_dest_simple(init_pos,
                                 dis_tol=self.dist_tol,
                                 angle_tol=self.angle_tol):
            self.crouch()
            print('go to init pos done[in GoToInitPos]')
            return Status.SUCCEEDED
        else:
            # self.goto_gobal(init_pos)
            self.bb.action_cmd.bodyCmd = BodyCommand(init_pos.x, init_pos.y,
                                                     init_pos.z, self.walk_type)
            print('going to init pos[in GoToInitPos]')
            self.bb.behavior_info.dest = init_pos
            self.bb.behavior_info.final_dest = init_pos

            return Status.RUNNING


class GoFuckPos(Skill):
    """Go to default position of role."""

    def execute(self):
        # if self.bb.lower_board_reconnected:
        #     self.crouch()
        #     return Status.RUNNING

        if self.get_bb().param.attack_right:
            init_pos = VecPos(130, 180, 45)
        else:
            init_pos = VecPos(-130, 180, -135)

        if self._got_dest_simple(init_pos, dis_tol=30, angle_tol=15):
            self.crouch()
            print('go to default pos done[in GoFuckPos]')
            return Status.SUCCEEDED
        else:
            self.goto_global(init_pos)
            print('going to default pos[in GoFuckPos]')
            self.bb.behavior_info.dest = init_pos
            self.bb.behavior_info.final_dest = init_pos

            return Status.RUNNING


class GoToPos(Skill):
    """Go to given position."""

    def __init__(self, bb, pos):
        """
        Initialization

        :param bb: blackboard
        :type bb: DBlackboard
        :param pos: given position
        :type pos: VecPos
        """
        super(GoToPos, self).__init__(bb)
        self.pos = pos

    def execute(self):
        pos = self.pos
        self.bb.behavior_info.dest = pos
        self.bb.behavior_info.final_dest = pos
        if self._got_dest_simple(self.pos, dis_tol=30, angle_tol=15):
            self.crouch()
            return Status.SUCCEEDED
        else:
            self.goto_global(self.pos)
            return Status.RUNNING


# class GoStraight(Skill):

#     def execute(self):
#         self.walk(5, 0, 0)
#         return Status.RUNNING


class InitialEntry(DynamicGuardSelector):
    """Entry at initial time for GC."""

    def __init__(self, bb):
        children = [
            # (Parallel(InitialParticles(bb),
            #           EntryScanField(bb)), GCNormalInitial()),
            (Parallel(Do(bb), FindBall(bb, look_down=True)), GCPenalised()),
            (EntryScanField(bb), GCNormalInitial()),
            (
                Parallel(
                    DynamicGuardSelector(
                        [
                            # (TurnAround(bb,
                            #             target_angle=90,
                            #             look_down=False,
                            #             follow_ball=False,
                            #             walk_backward=True),
                            #  FarFromInitPos()),
                            (GoToInitPos(bb),
                             GotOutOfInitPos(dis_tol=10, angle_tol=10),
                             GotInitPos(dis_tol=10))
                        ],
                        use_exit=True),
                    DynamicGuardSelector([(TrackCircle(bb), CircleSeen()),
                                          (TrackGoal(bb), GoalSeen()),
                                          (Do(bb, cmd='look',
                                              pitch=10), None)])),
                #   (ScanFieldNew(bb), None)])),
                GCNormalReady()),
            # add proper behavior for penalised robot
            (Parallel(Do(bb), FindBall(bb, look_down=True)),
             GuardSelector(GCNormalSet())),  # GCPenalised()
        ]
        super(InitialEntry, self).__init__(children)


class ReEntry(DynamicGuardSelector):
    """Re-entry for GC."""

    def __init__(self, bb):
        children = [
            (Parallel(GoToInitPos(bb), ScanField(bb)), None),
        ]
        super(ReEntry, self).__init__(children)
