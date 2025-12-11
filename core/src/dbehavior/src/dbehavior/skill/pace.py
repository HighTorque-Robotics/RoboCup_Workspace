from dbehavior.core import Skill, Role, Do
from dbehavior.skill import SeekBall, TrackBall, FindBall
from dbehavior.util import VecPos
from dbehavior.util.mathutil import get_dis, degree_between, get_magnitude, abs_angle_diff
from dbehavior.btree.core import Status
from dmsgs.msg import BehaviorInfo
from dbehavior.btree.branch import Parallel, DynamicGuardSelector
from dbehavior.btree.leaf import BallSeen, ConditionLeaf
import rospy


class PaceBall(Skill):

    def __init__(self, bb):
        super(PaceBall, self).__init__(bb)
        self.assist_radius = 120
        self.pass_dis = 270
        self.far_from_ball = 80
        self.shoot_ball = False
        self.angle_per_turn = 8

    def execute(self):
        if self.bb.see_ball:
            ball_global = VecPos.from_vector3(self.bb.ball_global)
        else:
            return Status.FAILED
        if self.bb.mate_target is None:
            target = self.bb.attack_target
        else:
            target = VecPos.from_vector3(self.bb.mate_target)
        g2b = ball_global - target

        # Ball is too far from the goal
        if not self.shoot_ball:
            robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
            offset = ball_global - robot_pos
            offset.x = offset.x / get_magnitude(offset) * 150
            offset.y = offset.y / get_magnitude(offset) * 150
            dest = ball_global - offset
            dest.z = degree_between(ball_global, robot_pos)
            if self._got_dest_simple(dest, dis_tol=100, angle_tol=50):
                self.turn(self.angle_per_turn)
                return Status.RUNNING
            self.goto_global(dest)
            return Status.RUNNING


        self.bb.behavior_info.dest = dest
        self.goto_global(dest)

        return Status.RUNNING


class GotPace(ConditionLeaf):

    def __init__(self):
        super(GotPace, self).__init__()

    def condition(self):
        return self.get_bb().param.pace




