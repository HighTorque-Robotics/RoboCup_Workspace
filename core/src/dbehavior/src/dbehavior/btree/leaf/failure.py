from dbehavior.btree.core import Leaf, Status


class Failure(Leaf):
    """
    ``Failure`` is a leaf that immediately fails.
    """

    def execute(self):
        """
        Executes this ``Failure`` task.

        :return: Failure status
        :rtype: Status
        """
        return Status.FAILED
