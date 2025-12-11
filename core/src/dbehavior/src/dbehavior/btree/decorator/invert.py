from dbehavior.btree.core import Decorator


class Invert(Decorator):
    """
    An ``Invert`` decorator will succeed if the wrapped task fails and will fail if the wrapped task succeeds.
    """

    def on_child_success(self, task):
        super(Invert, self).on_child_failure(task)

    def on_child_failure(self, task):
        super(Invert, self).on_child_success(task)

    def get_running_route(self):
        if self.child is None:
            return "{}".format(self.__repr__())
        else:
            return "-{}".format(self.child.__repr__())
