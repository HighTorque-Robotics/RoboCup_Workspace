from dbehavior.btree.core import Branch


class DynamicGuardSelector(Branch):
    """
    A ``DynamicGuardSelector`` is a branch task that executes the first child
    whose guard is evaluated to ``True``. At every AI cycle, the children's
    guards are re-evaluated, so if the guard of the running child is evaluated
    to ``False``, it is cancelled, and the child with the highest priority
    starts running. The ``DynamicGuardSelector`` task finishes when no guard
    is evaluated to ``True`` (thus failing) or when its active child finishes
    (returning the active child's termination status).
    """

    def __init__(self, children, buffer_size=1, use_exit=False):
        """
        Init DynamicGuardSelector.

        :param children: children with guard
        :type children: list
        """
        super(DynamicGuardSelector, self).__init__()
        self._child_to_run = None

        self.use_exit = use_exit
        if not self.use_exit:
            for child, guard in children:
                if guard is not None:
                    child.set_guard(guard)
                self.add_child(child)
        else:
            for child, guard, exit_cond in children:
                if guard is not None:
                    child.set_guard(guard)
                if exit_cond is not None:
                    child.set_exit(exit_cond)
                self.add_child(child)

        self.buffer_index = 0
        self.buffer_size = buffer_size
        self.buffer = [None] * self.buffer_size

    def append_buffer(self, child_to_run):
        self.buffer[self.buffer_index] = child_to_run
        self.buffer_index = (self.buffer_index + 1) % self.buffer_size

    def check_buffer(self):
        if None in self.buffer:
            return False

        switch_running_child = True
        for child in self.buffer:
            if child is not None and child != self.buffer[0]:
                switch_running_child = False
        return switch_running_child

    def clear_buffer(self):
        self.buffer_index = 0
        self.buffer = [None] * self.buffer_size

    def run(self):
        self._child_to_run = None
        for idx, child in enumerate(self._children):
            # print('{}: [guard]{} [exit]{}'.format(child,
            #                                       child.check_guard(),
            #                                       child.check_exit()))
            if child.check_guard():
                self._child_to_run = idx
                self.append_buffer(self._child_to_run)
                break

        if self.check_buffer() and self._running_child is not None:
            current_running_child = self.get_child(self._running_child)
            if current_running_child.check_exit() and \
                    self._running_child != self._child_to_run:
                current_running_child.cancel()
                self._running_child = None
                self.clear_buffer()

        if self._child_to_run is None:
            self.failure()
        else:
            if self._running_child is None:
                self._running_child = self._child_to_run
                running_child = self.get_child(self._running_child)
                running_child.set_control(self)
                running_child.on_start()
            else:
                running_child = self.get_child(self._running_child)
            running_child.run()

    def reset(self):
        super(DynamicGuardSelector, self).reset()
        self._running_child = None

    def on_child_running(self, task):
        self._running_child = self._children.index(task)
        self.running()

    def on_child_success(self, task):
        self._running_child = None
        self.success()

    def on_child_failure(self, task):
        self._running_child = None
        self.failure()

    def get_running_child(self):
        """
        Returns index of current running child.
        :return: index of current running child
        :rtype: Task
        """
        return self._running_child

    def get_running_route(self):
        if self._child_to_run is None:
            return "{}".format(self.__repr__())
        else:
            child = self.get_child(self._child_to_run)
            guard = child.get_guard()
            if guard is not None:
                return "[{}]->".format(guard.get_running_route()) + child.get_running_route()
            else:
                return child.get_running_route()
