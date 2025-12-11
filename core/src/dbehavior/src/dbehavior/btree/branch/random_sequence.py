from dbehavior.btree.branch import Sequence


class RandomSequence(Sequence):
    """
    A ``RandomSequence`` is a sequence task's variant that runs its children in a random order.
    """

    def on_start(self):
        super(RandomSequence, self).on_start()
        if self._random_children is None:
            self._random_children = self.create_random_children()

