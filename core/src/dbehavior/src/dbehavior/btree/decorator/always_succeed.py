from dbehavior.btree.core import Decorator


class AlwaysSucceed(Decorator):
    """
    An ``AlwaysSucceed`` decorator will succeed no matter the wrapped task succeeds or fails.
    """

    def on_child_failure(self, task):
        self.on_child_success(task)
