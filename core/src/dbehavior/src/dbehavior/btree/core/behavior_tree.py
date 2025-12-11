from dbehavior.btree.core import Task, Status, Blackboard


class BehaviorTree(Task):
    """
    The behavior tree itself.
    """

    def __init__(self, bb):
        super(BehaviorTree, self).__init__()
        self._tree = self
        self.root_task = None
        self.guard_evaluator = GuardEvaluator()
        self.guard_evaluator.set_control(self)
        self.exit_evaluator = GuardEvaluator()
        self.exit_evaluator.set_control(self)

        self._bb = None
        self.set_bb(bb)

    def add_child(self, child):
        if self.root_task is not None:
            raise RuntimeError('A behavior tree cannot have more than one root task')
        child.set_control(self)
        self.root_task = child
        return 0

    def get_child_count(self):
        if self.root_task is None:
            return 0
        else:
            return 1

    def get_child(self, idx):
        if idx == 0 and self.root_task is not None:
            return self.root_task
        else:
            raise IndexError('Invalid index {} for child'.format(idx))

    def on_child_running(self, task):
        self.running()

    def on_child_success(self, task):
        self.success()

    def on_child_failure(self, task):
        self.failure()

    def reset(self):
        super(BehaviorTree, self).reset()
        self.root_task = None

    def run(self):
        self.step()

    def step(self):
        """
        This method should be called when game entity needs to make decisions:
        call this in game loop or after a fixed time slice if the game is
        real-time, or on entity's turn if the game is turn-based.
        """
        if self.root_task.get_status() == Status.RUNNING:
            self.root_task.run()
        else:
            self.root_task.set_control(self)
            self.root_task.on_start()
            # print('33333333')
            if self.root_task.check_guard():
                # print('444')
                self.root_task.run()
                print('root_task.run() completed')
            else:
                self.root_task.failure()
                print('root_task.failure()')

    def get_bb(self):
        return self._bb

    def set_bb(self, bb):
        """
        This method will set a task as this task's blackboard.

        :param bb: the parent task
        :type bb: Blackboard
        """
        self._bb = bb

    def get_running_route(self):
        return "{}->".format(self.__repr__()) + self.root_task.get_running_route()


class GuardEvaluator(Task):
    """
    Shared evaluator for guard in behavior tree.
    """

    def add_child(self, child):
        return 0

    def get_child_count(self):
        return 0

    def get_child(self, idx):
        return None

    def on_child_running(self, task):
        pass

    def on_child_success(self, task):
        pass

    def on_child_failure(self, task):
        pass

    def run(self):
        pass


class ExitEvaluator(Task):
    """
    Shared evaluator for exit in behavior tree.
    """

    def add_child(self, child):
        return 0

    def get_child_count(self):
        return 0

    def get_child(self, idx):
        return None

    def on_child_running(self, task):
        pass

    def on_child_success(self, task):
        pass

    def on_child_failure(self, task):
        pass

    def run(self):
        pass
