from dbehavior.btree.core import Decorator


class AlwaysFail(Decorator):
    """
    An ``AlwaysFail`` decorator will fail no matter the wrapped task fails or succeeds.
    """

    def on_child_success(self, task):
        self.on_child_failure(task)
