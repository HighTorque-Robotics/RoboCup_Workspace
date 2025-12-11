from dbehavior.btree.branch import Selector


class RandomSelector(Selector):
    """
    A ``RandomSelector`` is a selector task's variant that runs its children in a random order.
    """

    def on_start(self):
        super(RandomSelector, self).on_start()
        if self._random_children is None:
            self._random_children = self.create_random_children()
