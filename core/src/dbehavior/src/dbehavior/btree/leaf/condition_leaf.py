from dbehavior.btree.core import Leaf, Status


class ConditionLeaf(Leaf):

    def condition(self):
        raise NotImplementedError

    def execute(self):
        if self.condition():
            return Status.SUCCEEDED
        else:
            return Status.FAILED
