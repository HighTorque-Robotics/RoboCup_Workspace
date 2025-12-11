from dbehavior.btree.core import Task, Status


class Branch(Task):
    """
    A branch task defines a behavior tree branch, contains logic of starting or running sub-branches and leaves.
    """

    def __init__(self):
        super(Branch, self).__init__()
        # The children of this task
        self._children = []
        # The index of child in the running status or ``None`` if no child is running.
        self._running_child = None

    def add_child(self, child):
        if not isinstance(child, Task):
            raise TypeError('{} is not an instance or subclass of Task'.format(type(child)))
        if child is self:
            raise TypeError('Trying to add self as child')

        child.set_control(self)
        self._children.append(child)

        return self.get_child_count() - 1

    def get_child_count(self):
        return len(self._children)

    def get_child(self, idx):
        if 0 <= idx <= self.get_child_count():
            return self._children[idx]
        else:
            raise IndexError('Invalid index {} for child'.format(idx))

    def get_running_route(self):
        if self._running_child is None:
            return "{}".format(self.__repr__())
        else:
            return "{}->".format(self.__repr__()) + self.get_child(self._running_child).get_running_route()
