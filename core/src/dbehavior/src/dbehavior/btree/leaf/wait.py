from dbehavior.btree.core import Leaf, Status
import time


class Wait(Leaf):
    """
    ``Wait`` is a leaf that keeps running for the specified amount of time then succeeds.
    """

    def __init__(self, timeout=0):
        """
        Initialize ``Wait`` task.

        :param timeout: time te wait in seconds
        :type timeout: int or float
        """
        super(Wait, self).__init__()
        self.start_time = None
        self.timeout = timeout

    def on_start(self):
        self.start_time = time.time()

    def execute(self):
        if self.start_time is None:
            self.on_start()

        if time.time() - self.start_time < self.timeout:
            return Status.RUNNING
        else:
            return Status.SUCCEEDED

    def reset(self):
        super(Wait, self).reset()
        self.start_time = None


class CountDown(Wait):
    """Succeed until timeout."""

    def on_start(self):
        if self.start_time is None:
            self.start_time = time.time()


def execute(self):
    if time.time() - self.start_time < self.timeout:
        return Status.SUCCEEDED
    else:
        return Status.FAILED
