from dbehavior.btree.core import SingleRunningChildBranch


class Sequence(SingleRunningChildBranch):
    """
    A ``Sequence`` is a branch task that runs every children until one of
    them fails. If a child task succeeds, the selector will start and run
    the next child task.
    """

    def __init__(self, *args):
        super(Sequence, self).__init__()
        for child in args:
            self.add_child(child)

    def on_child_success(self, task):
        if self._current_child_index + 1 < self.get_child_count():
            self._current_child_index += 1
            self._running_child = None
            self.running()
            # self.run()
        else:
            self._current_child_index = 0
            self._running_child = None
            self.success()

    def on_child_failure(self, task):
        self.failure()


class GuardSequence(Sequence):

    def on_child_success(self, task):
        if self._current_child_index + 1 < self.get_child_count():
            self._current_child_index += 1
            self._running_child = None
            self.run()
        else:
            self._current_child_index = 0
            self._running_child = None
            self.success()
