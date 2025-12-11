from dbehavior.btree.decorator import Condition


class SeeBall(Condition):
    """
    The ``SeeBall`` decorator will return status by the result of see ball.
    """

    def condition(self):
        return self.get_bb().vision_info.see_ball
