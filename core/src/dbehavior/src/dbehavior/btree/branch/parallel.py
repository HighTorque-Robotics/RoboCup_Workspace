from dbehavior.btree.core import Branch, Status
from enum import Enum


class ParallelPolicy(Enum):
    """
    The enumeration of the policies supported by the ``Parallel`` task.
    """
    # The sequence policy makes the ``Parallel`` task fail as soon as
    # one child fails; if all children succeed, then the parallel task
    # succeeds. This is the default policy.
    SEQUENCE = 0
    # he selector policy makes the ``Parallel`` task succeed
    # as soon as one child succeeds; if all children fail,
    # then the parallel task fails.
    SELECTOR = 1


class ParallelOrchestrator(Enum):
    """
    The enumeration of the child orchestrators supported by the ``Parallel`` task.
    """
    # The default orchestrator - starts or resumes all children every single step
    RESUME = 0
    # Children execute until they succeed or fail but will not re-run
    # until the parallel task has succeeded or failed
    JOIN = 1


class Parallel(Branch):
    """
    A ``Parallel`` is a special branch task that runs all children
    when stepped. Its actual behavior depends on its ``orchestrator``
    and ``policy``.

    The execution of the parallel task's children depends on its ``orchestrator``:

    - ``Orchestrator#Resume``: the parallel task restarts or runs each child every step
    - ``Orchestrator#Join``: child tasks will run until success or failure but
        will not re-run until the parallel task has succeeded or failed

    The actual result of the parallel task depends on its ``policy``:

    - ``Policy#Sequence``: the parallel task fails as soon as one child fails;
        if all children succeed, then the parallel task succeeds.
        This is the default policy.
    - ``Policy#Selector``: the parallel task succeeds as soon as one child
        succeeds; if all children fail, then the parallel task fails.

    The typical use case: make the game entity react on event while sleeping or wandering.
    """

    def __init__(self, *args):
        """
        Initialize parallel branch.
        """
        super(Parallel, self).__init__()

        policy = ParallelPolicy.SEQUENCE
        orchestrator = ParallelOrchestrator.RESUME

        self._policy = policy
        self._orchestrator = orchestrator

        self._no_running_tasks = True
        self._last_result = None
        self._current_child_index = 0

        for child in args:
            self.add_child(child)

    def reset(self):
        super(Parallel, self).reset()
        self._policy = ParallelPolicy.SEQUENCE
        self._orchestrator = ParallelOrchestrator.RESUME

        self._no_running_tasks = True
        self._last_result = None
        self._current_child_index = 0

    def run(self):
        if self._orchestrator == ParallelOrchestrator.RESUME:
            self._no_running_tasks = True
            self._last_result = None
            for idx in range(self.get_child_count()):
                self._current_child_index = idx
                child = self.get_child(self._current_child_index)
                if child.get_status() == Status.RUNNING:
                    child.run()
                else:
                    child.set_control(self)
                    child.on_start()
                    if child.check_guard():
                        child.run()
                    else:
                        child.failure()

                # Current child has finished either with success or fail
                if self._last_result is not None:
                    child_to_cancel = self._current_child_index + 1 \
                        if self._no_running_tasks else 0
                    self.cancel_running_children(child_to_cancel)

                    if self._last_result:
                        self.success()
                    else:
                        self.failure()
                    return
                self.running()
        elif self._orchestrator == ParallelOrchestrator.JOIN:
            self._no_running_tasks = True
            self._last_result = None
            for idx in range(self.get_child_count()):
                self._current_child_index = idx
                child = self.get_child(self._current_child_index)
                if child.get_status() == Status.RUNNING:
                    child.run()
                elif child.get_status() == Status.SUCCEEDED or \
                        child.get_status() == Status.FAILED:
                    continue
                else:
                    child.set_control(self)
                    child.on_start()
                    if child.check_guard():
                        child.run()
                    else:
                        child.failure()

                # Current child has finished either with success or fail
                if self._last_result is not None:
                    child_to_cancel = self._current_child_index + 1 \
                        if self._no_running_tasks else 0
                    self.cancel_running_children(child_to_cancel)

                    if self._last_result:
                        self.success()
                    else:
                        self.failure()
                    return
                self.running()

    def on_child_running(self, task):
        self._no_running_tasks = False

    def on_child_success(self, task):
        if self._policy == ParallelPolicy.SEQUENCE:
            if self._orchestrator == ParallelOrchestrator.RESUME:
                if self._no_running_tasks and \
                        self._current_child_index == self.get_child_count() - 1:
                    self._last_result = True
            elif self._orchestrator == ParallelOrchestrator.JOIN:
                if self._no_running_tasks and \
                        self.get_child(self.get_child_count() - 1).get_status() == Status.SUCCEEDED:
                    self._last_result = True
        elif self._policy == ParallelPolicy.SELECTOR:
            self._last_result = True

    def on_child_failure(self, task):
        if self._policy == ParallelPolicy.SEQUENCE:
            self._last_result = False
        elif self._policy == ParallelPolicy.SELECTOR:
            if self._no_running_tasks and \
                    self._current_child_index == self.get_child_count() - 1:
                self._last_result = False

    def get_running_route(self):
        if self._current_child_index is None:
            return "{}".format(self.__repr__())
        else:
            return self.get_child(self._current_child_index).get_running_route()
