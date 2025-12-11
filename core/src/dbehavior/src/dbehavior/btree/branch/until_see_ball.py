from dbehavior.btree.core import LoopTickDecorator


class UntilSeeBall(LoopTickDecorator):
    """
    The ``UntilSeeBall`` decorator will repeat the wrapped task until robot has
    seen the ball, which makes the decorator succeed.
    """

    def condition(self):
        return not self.get_bb().vision_info.see_ball
