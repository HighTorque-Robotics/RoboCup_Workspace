from dbehavior.btree.core import Leaf, Status
import time


class Period(Leaf):
    """
    ``Period`` is a leaf that works periodly.
    """

    def __init__(self, timeout=0, timekeep=3):
        super(Period, self).__init__()
        self.start_time = time.time()
        self.keep_time = None
        self.timeout = timeout
        self.timekeep = timekeep
        self.keep = False
        self.initialized = False

    def execute(self):
        if not self.initialized:
            self.start_time = time.time()
            self.initialized = True

        # print(self.keep)
        # print(time.time() - self.start_time)

        if self.keep:
            # print(time.time() - self.keep_time)
            if time.time() - self.keep_time < self.timekeep:
                return Status.SUCCEEDED
            else:
                self.keep = False
                self.start_time = time.time()

        if time.time() - self.start_time < self.timeout:
            return Status.FAILED
        else:
            self.keep = True
            self.keep_time = time.time()
            return Status.SUCCEEDED
