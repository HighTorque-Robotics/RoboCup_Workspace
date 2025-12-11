from dbehavior.btree.core import Decorator
from random import random


class Random(Decorator):
    """
    The ``Random`` decorator succeeds with the specified probability,
    regardless of whether the wrapped task fails or succeeds. Also,
    the wrapped task is optional, meaning that this decorator can
    act like a leaf task.

    Notice that if success probability is 1 this task is equivalent
    to the decorator ``AlwaysSucceed`` and the leaf ``Success``.
    Similarly if success probability is 0 this task is equivalent
    to the decorator ``AlwaysFail`` and the leaf ``Failure``.
    """

    def __init__(self, child=None, probability=0):
        super(Random, self).__init__(child)
        self.p = probability

    def on_start(self):
        """
        Draws a value from the distribution that determines the success
        probability. This method is called when the task is entered.
        """
        self.p = random()

    def run(self):
        if self.child is not None:
            super(Random, self).run()
        else:
            self.decide()

    def on_child_success(self, task):
        self.decide()

    def on_child_failure(self, task):
        self.decide()

    def decide(self):
        """
        Decides whether succeeds or fails by generate a random float numer
        and compare it with given probability.
        """
        if random() <= self.p:
            self.success()
        else:
            self.failure()

    def reset(self):
        super(Random, self).reset()
        self.p = 0
