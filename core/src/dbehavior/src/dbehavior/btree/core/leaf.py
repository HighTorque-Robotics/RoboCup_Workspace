from dbehavior.btree.core import Task, Status


class Leaf(Task):
    """
    A ``Leaf Task`` is a terminal task of a behavior tree, contains action or condition logic, can not have any child.
    """

    def add_child(self, child):
        raise RuntimeError('A leaf task cannot have any children!')

    def get_child_count(self):
        return 0

    def get_child(self, idx):
        raise RuntimeError('A leaf task does not have any children!')

    def execute(self):
        """
        This method contains the update logic of this leaf task.

        :return: the status of this leaf task
        :rtype: Status
        """
        raise NotImplemented('Method is not implemented!')

    def run(self):
        result = self.execute()
        if result is None:
            raise RuntimeError("Get none status of leaf task")
        if result == Status.SUCCEEDED:
            print('executing {}: SUCCEEDED'.format(self.__class__.__name__))
            self.success()
        elif result == Status.FAILED:
            print('executing {}: FAILED'.format(self.__class__.__name__))
            self.failure()
        elif result == Status.RUNNING:
            print('executing {}: RUNNING'.format(self.__class__.__name__))
            self.running()
            print('completed {}: RUNNING'.format(self.__class__.__name__))
        else:
            raise RuntimeError('Invalid status {} returned by the execute() method'.format(result))

    def on_child_running(self, task):
        pass

    def on_child_success(self, task):
        pass

    def on_child_failure(self, task):
        pass

    def get_running_route(self):
        return self.__repr__()
