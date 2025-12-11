from dbehavior.btree.leaf import ConditionLeaf

class CircleSeen(ConditionLeaf):
    def condition(self):
        return self.get_bb().circle_seen

class GoalSeen(ConditionLeaf):
    def condition(self):
        return self.get_bb().goal_seen