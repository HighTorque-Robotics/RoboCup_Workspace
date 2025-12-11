from enum import Enum
from dbehavior.btree.core import Blackboard


class Status(Enum):
    """The enumeration of the values that a task's status can have."""
    # Means that the task has never run or has been reset.
    FRESH = 0
    # Means that the task needs to run again.
    RUNNING = 1
    # Means that the task returned a failure result.
    FAILED = 2
    # Means that the task returned a success result.
    SUCCEEDED = 3
    # Means that the task has been terminated by an ancestor.
    CANCELLED = 4


class Task(object):
    """The abstract base class of all behavior tree tasks."""

    def __init__(self):
        """Initialize task"""
        # The status of this task
        self._status = Status.FRESH
        # The behavior tree this task belongs to
        self._tree = None
        # The parent of this task
        self._control = None
        # The guard of this task
        self._guard = None
        # the exit status og this task
        self._exit = None

    def __repr__(self):
        return self.__class__.__name__

    def add_child(self, child):
        """
        This method will add a child to the list of this task's children

        :param child: the child task which will be added
        :type child: Task
        :return: the index where the child has been added
        :rtype: int
        """
        raise NotImplemented('Method is not implemented!')

    def get_child_count(self):
        """
        Returns the number od children of this task.

        :return: an int giving the number of children of this task
        :rtype: int
        """
        raise NotImplemented('Method is not implemented!')

    def get_child(self, idx):
        """
        Returns the child at the given index.

        :param idx: index of child
        :type idx: int
        :return: child by index
        :rtype: Task
        """
        raise NotImplemented('Method is not implemented!')

    def get_status(self):
        """
        Returns the status of this task.

        :return: the status
        :rtype: Task
        """
        return self._status

    def get_tree(self):
        """
        Returns the behavior tree this task belongs to.

        :return: the behavior tree
        :rtype: Task
        """
        if self._tree is None and \
                self._control is not None and \
                self._control.get_tree() is not None:
            self._tree = self._control.get_tree()
        return self._tree

    def get_bb(self):
        """
        Returns the blackboard of this task.

        :return: the blackboard
        :rtype: Blackboard or DBlackboard
        """
        if self.get_tree() is None:
            raise RuntimeError('This task is has never run')
        return self.get_tree().get_bb()

    def get_control(self):
        """
        Returns the control of this task.

        :return: the control
        :rtype: Task
        """
        return self._control

    def set_control(self, control):
        """
        This method will set a task as this task's control (parent).

        :param control: the parent task
        :type control: Task
        """
        self._control = control
        self._tree = control.get_tree()

    def set_guard(self, guard):
        """
        Sets the guard of this task.

        :param guard: the guard
        :type guard: Task
        """
        self._guard = guard

    def get_guard(self):
        """
        Returns the guard of this task.

        :return: the guard
        :rtype: Task
        """
        return self._guard

    def check_guard(self):
        """
        Checks the guard of this task.

        :return: ``True`` if guard evaluation succeeds or there's no guard; ``False`` otherwise.
        """
        # No guard to check
        if self._guard is None:
            # print('55555')
            return True
        # Check the guard of the guard recursively
        if not self._guard.check_guard():
            return False
        # Use the tree's guard evaluator task to check the guard of this task
        self._guard.set_control(self._control.get_tree().guard_evaluator)
        self._guard.on_start()
        self._guard.run()
        if self._guard.get_status() == Status.SUCCEEDED:
            return True
        elif self._guard.get_status() == Status.FAILED:
            return False
        else:
            raise RuntimeError(
                'Invalid guard status {}. Guards must either succeed or fail in one step.'.format(result))

    def set_exit(self, exit):
        """
        Sets the exit of this task.

        :param exit: the exit
        :type exit: Task
        """
        self._exit = exit

    def get_exit(self):
        """
        Returns the exit of this task.

        :return: the exit
        :rtype: Task
        """
        return self._exit

    def check_exit(self):
        """
        Checks the exit of this task.

        :return: ``True`` if guard evaluation succeeds or there's no exit; ``False`` otherwise.
        """
        # No exit to check
        if self._exit is None:
            return True
        # Check the exit of the guard recursively
        if not self._exit.check_exit():
            return False
        # Use the tree's guard evaluator task to check the guard of this task
        self._exit.set_control(self._control.get_tree().exit_evaluator)
        self._exit.on_start()
        self._exit.run()
        if self._exit.get_status() == Status.SUCCEEDED:
            return True
        elif self._exit.get_status() == Status.FAILED:
            return False
        else:
            raise RuntimeError(
                'Invalid exit status {}. Guards must either succeed or fail in one step.'.format(result))

    def on_start(self):
        """
        This method should be called once before this task's first run.
        """
        pass

    def on_end(self):
        """
        This method will be called by ``success()``, ``fail()`` or ``cancel()``
        meaning that this task's status has just been set to
        ``Status.SUCCEEDED``, ``Status.FAILED`` or  ``Status.CANCELLED``
        respectively.
        """
        pass

    def run(self):
        """
        This method contains the update logic of this task.
        The actual implementation MUST call ``running()``
        """
        raise NotImplemented('Method is not implemented!')

    def running(self):
        """
        This method will be called in ``run()`` to inform control that this task needs to run again.
        """
        # print("{} running".format(self))
        self._status = Status.RUNNING
        if self._control is not None:
            self._control.on_child_running(self)

    def success(self):
        """
        This method will be called in ``run()`` to inform control that this task has finished running with a success result.
        """
        # print("{} succeed".format(self))
        self._status = Status.SUCCEEDED
        self.on_end()
        if self._control is not None:
            self._control.on_child_success(self)

    def failure(self):
        """
        This method will be called in ``run()`` to inform control that this task has finished running with a failure result.
        """
        # print("{} failed".format(self))
        self._status = Status.FAILED
        self.on_end()
        if self._control is not None:
            self._control.on_child_failure(self)

    def cancel(self):
        """
        Terminates this task and all its running children. This method MUST be called only if this task is running.
        """
        self._status = Status.CANCELLED
        self.on_end()

    def reset(self):
        """
        Resets this task to make it restart from scratch on next run.
        """
        if self._status == Status.RUNNING:
            self.cancel()
        for idx in range(self.get_child_count()):
            child = self.get_child(idx)
            child.reset()

        self._status = Status.FRESH
        self._guard = None

    def on_child_running(self, task):
        """
        This method will be called when one of the ancestors of this task needs to run again

        :param task: the task that needs to run again
        :type task: Task
        """
        raise NotImplemented('Method is not implemented!')

    def on_child_success(self, task):
        """
        This method will be called when one of the children of this task succeeds.

        :param task: the task that succeeded
        :type task: Task
        """
        raise NotImplemented('Method is not implemented!')

    def on_child_failure(self, task):
        """
        This method will be called when one of the children of this task fails.

        :param task: the task that failed
        :type task: Task
        """
        raise NotImplemented('Method is not implemented!')

    def cancel_running_children(self, start_index):
        """
        Terminates the running children of this task starting from the specified index up to the end.

        :param start_index: the start index
        :type start_index: int
        """
        for idx in range(start_index, self.get_child_count()):
            child = self.get_child(idx)
            if child.get_status() == Status.RUNNING:
                child.cancel()

    def get_running_route(self):
        """
        Return route from this node to running leaf.

        :return: running route
        :rtype: str
        """
        raise NotImplemented('Method is not implemented!')

