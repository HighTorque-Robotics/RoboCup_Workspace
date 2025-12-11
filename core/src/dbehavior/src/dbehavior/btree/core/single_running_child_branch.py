from dbehavior.btree.core import Branch
from random import randint


class SingleRunningChildBranch(Branch):
    """
    A ``SingleRunningChildBranch`` task is a branch task that supports
    only one running child at a time.
    """

    def __init__(self):
        super(SingleRunningChildBranch, self).__init__()
        # The index of the child currently processed.
        self._current_child_index = 0
        # Array of random children. If it's ``None`` this task is deterministic.
        self._random_children = None

    def on_child_running(self, task):
        self._running_child = self._children.index(task)
        self.running()

    def on_child_success(self, task):
        self._running_child = None

    def on_child_failure(self, task):
        self._running_child = None

    def run(self):
        if self._running_child is not None:
            child = self.get_child(self._running_child)
            child.run()
        else:
            if self._current_child_index < self.get_child_count():
                if self._random_children is not None:
                    last = self.get_child_count() - 1
                    if self._current_child_index < last:
                        other_child_index = randint(self._current_child_index, last)
                        tmp_child = self._random_children[self._current_child_index]
                        self._random_children[self._current_child_index] = self._random_children[other_child_index]
                        self._random_children[other_child_index] = tmp_child
                    self._running_child = self._children.index(self._random_children[self._current_child_index])
                else:
                    self._running_child = self._current_child_index

                child = self.get_child(self._running_child)
                child.set_control(self)
                child.on_start()
                if not child.check_guard():
                    child.failure()
                else:
                    self.run()
            else:
                # Should never happen; this case must be handled by subclasses in childXXX methods
                pass

    def on_start(self):
        self._current_child_index = 0
        self._running_child = None

    def cancel_running_children(self, start_index):
        super(SingleRunningChildBranch, self).cancel_running_children(start_index)
        self._running_child = None

    def reset(self):
        super(SingleRunningChildBranch, self).reset()
        self._running_child = None
        self._current_child_index = 0
        self._random_children = None

    def create_random_children(self):
        self._random_children = self._children[:]

    def get_running_route(self):
        if self._running_child is None:
            return self.get_child(self._current_child_index).get_running_route()
        else:
            return self.get_child(self._running_child).get_running_route()
