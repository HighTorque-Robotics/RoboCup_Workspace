from dbehavior.btree.core import Leaf, Status


class Success(Leaf):
    """
    ``Success`` is a leaf that immediately succeed.
    """

    def execute(self):
        """
        Executes this ``Success`` task.

        :return: Success status
        :rtype: Status
        """
        return Status.SUCCEEDED
