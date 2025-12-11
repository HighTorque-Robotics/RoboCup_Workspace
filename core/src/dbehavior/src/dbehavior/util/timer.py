import time


class Timer(object):
    """
    Timer.
    """
    def __init__(self, time_target=0, left_shift=0):
        """
        Initialize Timer.

        :param time_target: target time in seconds
        :type time_target: float
        """
        self._start = time.time()
        self.leftshift(left_shift)
        self._time_target = time_target

    def restart(self):
        """
        Restart timer.
        """
        self._start = time.time()

    def elapsed(self):
        """
        Get elapsed time since start as secs.

        :return: elapsed time.
        :rtype: float
        """
        return time.time() - self._start

    def finished(self):
        """
        Returns whether or not desired time duration has passed.

        :return: whether or not desired time duration has passed.
        :rtype: bool
        """
        return self.elapsed() >= self._time_target

    @staticmethod
    def sleep(duration):
        """
        Sleep for a while.

        :param duration: duration to sleep
        :type duration: float
        """
        time.sleep(duration)

    def leftshift(self, duration):
        """
        Leftshift timer.

        :param duration: duration to leftshift
        :type duration: float
        """
        self._start -= duration
