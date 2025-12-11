from dbehavior.btree.core import SingleRunningChildBranch


class Selector(SingleRunningChildBranch):
    """
    A ``Selector`` is a branch task that runs every children until one of them
    succeeds. If a child task fails, the selector will start and run the next
    child task.
    """

    def __init__(self, *args):
        super(Selector, self).__init__()
        for child in args:
            self.add_child(child)

    def on_child_failure(self, task):
        if self._current_child_index + 1 < self.get_child_count():
            self._current_child_index += 1
            self._running_child = None
            self.running()
            # self.run()
        else:
            self.failure()

    def on_child_success(self, task):
        # print('{} succeeds'.format(task))
        self.success()


class GuardSelector(Selector):
    def on_child_failure(self, task):
        if self._current_child_index + 1 < self.get_child_count():
            self._current_child_index += 1
            self._running_child = None
            self.run()
        else:
            self.failure()
