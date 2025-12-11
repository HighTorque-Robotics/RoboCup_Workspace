from dbehavior.btree.core import Decorator, Status
from dbehavior.btree.leaf import Success


class Condition(Decorator):
    """
    An ``Condition`` decorator will succeed only when the condition is satisfied.
    """

    def __init__(self, child=Success()):
        super(Condition, self).__init__(child)

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
            self.failure()

    def on_child_running(self, task):
        self.running()

    def on_child_success(self, task):
        if self.condition():
            self.success()
        else:
            self.failure()

    def on_child_failure(self, task):
        self.failure()


class ConditionOnce(Decorator):
    """
    An ``ConditionOnce`` decorator will succeed once the condition is satisfied.
    """

    def __init__(self, child=Success()):
        super(ConditionOnce, self).__init__(child)
        self.flag = False

    def on_start(self):
        self.flag = False

    def reset(self):
        self.flag = False

    def condition(self):
        raise NotImplementedError

    def run(self):
        if self.condition():
            self.flag = True

        if self.flag:
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
            self.failure()

    def on_child_running(self, task):
        self.running()

    def on_child_success(self, task):
        if self.flag:
            self.success()
        else:
            self.failure()

    def on_child_failure(self, task):
        self.failure()
