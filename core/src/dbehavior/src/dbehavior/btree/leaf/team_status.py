from dbehavior.btree.leaf import ConditionLeaf


class MateAttacking(ConditionLeaf):

    def condition(self):
        return self.get_bb().mate_attacking


class MateBallHandling(ConditionLeaf):

    def condition(self):
        return self.get_bb().mate_ball_handling


class TeamBallSeen(ConditionLeaf):
    def condition(self):
        return self.get_bb().param.ball_share_enabled and \
               self.get_bb().team_ball_seen
