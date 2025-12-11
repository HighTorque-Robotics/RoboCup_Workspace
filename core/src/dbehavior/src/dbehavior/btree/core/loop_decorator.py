from dbehavior.btree.core import Decorator, Status


class LoopDecorator(Decorator):
    """
    ``LoopDecorator`` is an abstract class providing basic functionalities
    for concrete looping decorators.
    """

    def __init__(self, child=None):
        super(LoopDecorator, self).__init__(child)
        self._loop = False

    def condition(self):
        return self._loop

    def run(self):
        self._loop = True
        while self.condition():
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
        super(LoopDecorator, self).on_child_running(task)
        self._loop = False

    def reset(self):
        super(LoopDecorator, self).reset()
        self._loop = False


class LoopTickDecorator(Decorator):
    """
    ``LoopTickDecorator`` is an abstract class providing basic functionalities
    for concrete looping ticking decorators.
    """

    def condition(self):
        raise NotImplementedError

    def run(self):
        if self.condition():
            if self.child.get_status() == Status.RUNNING:
                self.child.run()
            else:
                self.child.set_control(self)
                self.child.on_start()
                if self.child.check_guard():
                    self.child.run()
                else:
                    self.child.failure()
        else:
            self.child.on_end()
            self.success()

    def on_child_running(self, task):
        self.running()

    def on_child_success(self, task):
        if self.condition():
            self.running()
        else:
            self.child.on_end()
            self.success()

    def on_child_failure(self, task):
        self.on_child_success(task)
