from dbehavior.btree.core import LoopDecorator


class RepeatTimes(LoopDecorator):
    """
    A ``RepeatCounts`` decorator will repeat the wrapped task a certain number
    of times, possibly infinite. This task always succeeds when reaches
    the specified number of repetitions.
    """

    def __init__(self, child=None, times=0):
        super(RepeatTimes, self).__init__(child)
        self.times = times
        self.count = 0

    def on_start(self):
        self.count = self.times

    def condition(self):
        return self._loop and self.count != 0

    def on_child_success(self, task):
        if self.count > 0:
            self.count -= 1
        if self.count == 0:
            super(RepeatTimes, self).on_child_success(task)
            self._loop = False
        else:
            self._loop = True

    def on_child_failure(self, task):
        self.on_child_success(task)

    def reset(self):
        super(RepeatTimes, self).reset()
        self.count = 0
