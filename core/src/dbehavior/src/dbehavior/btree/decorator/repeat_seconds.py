from dbehavior.btree.core import LoopTickDecorator
from dbehavior.btree.core import Status
from time import time


class RepeatSeconds(LoopTickDecorator):
    """
    A ``RepeatSeconds`` decorator will repeat the wrapped task a certain number
    of seconds, possibly infinite. This task always succeeds when reaches
    the specified seconds.
    """

    def __init__(self, child=None, timeout=0):
        """
        Initialize RepeatSeconds.

        :param child: child to repeat
        :type child: Task
        :param timeout: timeout for repeating
        :type timeout: float or int
        """
        super(RepeatSeconds, self).__init__(child)
        self.timeout = timeout
        self.start_time = None

    def on_start(self):
        self.start_time = time()

    def condition(self):
        return time() - self.start_time < self.timeout

    def run(self):
        if self.start_time is None:
            self.on_start()
        super(RepeatSeconds, self).run()

    def reset(self):
        super(RepeatSeconds, self).reset()
        self.start_time = None
