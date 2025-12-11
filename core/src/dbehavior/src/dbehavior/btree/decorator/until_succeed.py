from dbehavior.btree.core import LoopDecorator


class UntilSucceed(LoopDecorator):
    """
    The ``UntilSucceed`` decorator will repeat the wrapped task until that task
    succeeds, which makes the decorator succeed.

    Notice that a wrapped task that always fails without entering the running
    status will cause an infinite loop in the current frame.
    """

    def on_child_success(self, task):
        self.success()
        self._loop = False

    def on_child_failure(self, task):
        self._loop = True
