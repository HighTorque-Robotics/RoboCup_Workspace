from dbehavior.btree.leaf import ConditionLeaf
from dbehavior.util.mathutil import get_magnitude
from dbehavior.util import VecPos
from math import fabs
import rospy
class BallSeen(ConditionLeaf):
    def condition(self):
        if self.get_bb().vision_info.see_ball:
            pass
            # rospy.loginfo('see ball!!!!!!!!!!')
        return self.get_bb().vision_info.see_ball


class BallLost(ConditionLeaf):
    def condition(self):
        return self.get_bb().ball_lost


class BallLostRecent(ConditionLeaf):

    def condition(self):
        return self.get_bb().ball_global_last and self.get_bb().ball_lost_recent


class BallInGoal(ConditionLeaf):
    """
    Check whether ball is in the enemy goal.
    """

    def condition(self):
        ball_global = self.get_bb().ball_global
        if ball_global is None:
            return False

        ball_global = self.get_bb().vision_info.ball_global
        if self.get_bb().param.attack_right:
            return ball_global >= 450
        else:
            return ball_global <= -450


class BallInOurPenaltyArea(ConditionLeaf):
    """
    Check whether ball is in penalty area.
    """

    def condition(self):
        # ball_global = self.get_bb().ball_global
        if not self.get_bb().vision_info.see_ball:
            return False
        ball_field = self.get_bb().vision_info.ball_field
        # if ball_global is None:
        if ball_field is None:
            return False
        return ball_field.x < 100
        # if self.get_bb().param.attack_right:
        #     return ball_global <= -350
        # else:
        #     return ball_global >= 350


class BallInOurHalf(ConditionLeaf):
    def condition(self):
        ball_global = self.get_bb().ball_global
        if ball_global is None:
            return False
        return (self.get_bb().param.attack_right and ball_global.x < 0) or \
               (not self.get_bb().param.attack_right and ball_global.x > 0)


class CloseToBall(ConditionLeaf):
    def __init__(self, dis=100):
        super(CloseToBall, self).__init__()
        self.dis = dis

    def condition(self):
        ball_field = self.get_bb().ball_field
        if ball_field is None:
            return False

        return get_magnitude(ball_field) <= self.dis

class FarFromBall(ConditionLeaf):
    def __init__(self, dis=100):
        super(FarFromBall, self).__init__()
        self.dis = dis

    def condition(self):
        ball_field = self.get_bb().ball_field
        if ball_field is None:
            return False

        return get_magnitude(ball_field) >= self.dis


class FarFromBall(ConditionLeaf):
    def __init__(self, dis=100):
        super(FarFromBall, self).__init__()
        self.dis = dis

    def condition(self):
        ball_field = self.get_bb().ball_field
        if ball_field is None:
            return False

        return get_magnitude(ball_field) >= self.dis


class ClosestToBall(ConditionLeaf):
    def condition(self):
        if not self.get_bb().vision_info.see_ball:
            return False
        return True
