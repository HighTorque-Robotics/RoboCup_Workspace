from dbehavior.btree.core import LoopDecorator


class UntilFail(LoopDecorator):
    """
    The ``UntilFail`` decorator will repeat the wrapped task until that task
    fails, which makes the decorator succeed.

    Notice that a wrapped task that always succeeds without entering
    the running status will cause an infinite loop in the current frame.
    """

    def on_child_success(self, task):
        self._loop = True

    def on_child_failure(self, task):
        self.success()
        self._loop = False
