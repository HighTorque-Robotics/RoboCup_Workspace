from dbehavior.btree.core import Task, Status


class Decorator(Task):
    """
    A ``Decorator`` is a wrapper that provides custom behavior for its child.
    """

    def __init__(self, child=None):
        super(Decorator, self).__init__()
        self.child = None
        self.add_child(child)

    def add_child(self, child):
        if child is None:
            return
        if not isinstance(child, Task):
            raise TypeError('{} is not an instance or subclass of Task'.format(type(child)))
        if self.get_child_count() >= 1:
            raise RuntimeError('Decorator should only have one child!')

        child.set_control(self)
        self.child = child

        return 0

    def get_child_count(self):
        if self.child is None:
            return 0
        else:
            return 1

    def get_child(self, idx):
        if idx == 0 and self.get_child_count() == 1:
            return self.child
        else:
            raise IndexError('Invalid index {} for child'.format(idx))

    def run(self):
        if self.child.get_status() == Status.RUNNING:
            self.child.run()
        else:
            self.child.set_control(self)
            self.child.on_start()
            if self.child.check_guard():
                self.child.run()
            else:
                self.child.failure()

    def on_child_running(self, task):
        self.running()

    def on_child_success(self, task):
        self.success()

    def on_child_failure(self, task):
        self.failure()

    def reset(self):
        super(Decorator, self).reset()
        self.child = None

    def get_running_route(self):
        if self.child is None:
            return "[{}]".format(self.__repr__())
        else:
            return "[{}]->".format(self.__repr__()) + self.child.get_running_route()


