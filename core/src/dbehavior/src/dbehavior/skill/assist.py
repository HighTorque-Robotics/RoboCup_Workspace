from dbehavior.core import Skill, Role, Do
from dbehavior.skill import SeekBall, TrackBall, FindBall
from dbehavior.util import VecPos
from dbehavior.util.mathutil import get_dis, degree_between, get_magnitude, abs_angle_diff
from dbehavior.btree.core import Status
from dmsgs.msg import BehaviorInfo
from dbehavior.btree.branch import Parallel, DynamicGuardSelector
from dbehavior.btree.leaf import BallSeen, ConditionLeaf
from dbehavior.skill import PaceBall, GotPace
import rospy


class GoToAssistPoint(Skill):

    def __init__(self, bb):
        super(GoToAssistPoint, self).__init__(bb)
        self.assist_radius = 120
        self.pass_dis = 270
        self.far_from_ball = 80
        self.shoot_ball = False
        self.stable_time = 3

    def execute(self):
        if self.bb.see_ball:
            ball_global = VecPos.from_vector3(self.bb.ball_global)
        else:
            return Status.FAILED
        if self.bb.mate_target is None:
            target = self.bb.attack_target
        else:
            target = VecPos.from_vector3(self.bb.mate_target)
        b2g = target - ball_global 
        if not self.shoot_ball:
            robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
            # next_ball_pos = ball_global + b2g.normalize(self.pass_dis)

            # dest = next_ball_pos

            offset = ball_global - robot_pos
            offset.x = offset.x / get_magnitude(offset) * 100
            offset.y = offset.y / get_magnitude(offset) * 100
            dest = ball_global - offset
            dest.z = degree_between(robot_pos, ball_global)
            self.bb.assist_point = dest
            # rospy.logfatal("calculate_assist")
            elapsed = self.bb.assist_timer.elapsed()
            if self._got_dest_simple(dest, dis_tol=100, angle_tol=50):
                self.crouch()
                self.bb.assist_timer.restart()
                return Status.RUNNING
            if elapsed > self.stable_time:
                self.goto_global(dest)
            return Status.RUNNING


class GotAssist(ConditionLeaf):

    def __init__(self):
        super(GotAssist, self).__init__()
        self.dis_tol = 100
        self.angle_tol = 50

    def condition(self):
        robot_pos = VecPos.from_vector3(self.get_bb().vision_info.robot_pos)

        if not self.get_bb().assist_point:
            print("\nNo assist point!\n")
            return False
        dis = get_dis(self.get_bb().assist_point, robot_pos)
        diff_angle = abs_angle_diff(self.get_bb().assist_point.z - robot_pos.z)

        return dis < self.dis_tol and diff_angle < self.angle_tol


class GotOutOfAssist(ConditionLeaf):

    def __init__(self):
        super(GotOutOfAssist, self).__init__()
        self.dis_tol = 100
        self.angle_tol = 50

    def condition(self):
        if self.get_bb().param.pace:
            return False
        robot_pos = VecPos.from_vector3(self.get_bb().vision_info.robot_pos)
        if not self.get_bb().assist_point:
            return True
        dis = get_dis(self.get_bb().assist_point, robot_pos)
        diff_angle = abs_angle_diff(self.get_bb().assist_point.z - robot_pos.z)

        return dis > self.dis_tol or diff_angle > self.angle_tol


class AssistBall(Parallel):

    def __init__(self, bb):
        args = (
            # GoToAssistPoint(bb),
            DynamicGuardSelector(
                [(GoToAssistPoint(bb), GotOutOfAssist(), GotAssist()),
                 (Do(bb), None, None)],
                use_exit=True),
            # DynamicGuardSelector(
            #     [(PaceBall(bb), GotPace())]),
            DynamicGuardSelector([
                (TrackBall(bb), BallSeen()),
                # If the ball position is provided by teammate, look ahead
                (FindBall(bb), None)
            ]))
        super(AssistBall, self).__init__(*args)

    def run(self):
        self.get_bb().set_state('assisting')
        super(AssistBall, self).run()

class PassBall(Parallel):

    def __init__(self, bb):
        args = (
            # GoToAssistPoint(bb),
            DynamicGuardSelector(
                [(GoToAssistPoint(bb), GotOutOfAssist(), GotAssist()),
                 (Do(bb), None, None)],
                use_exit=True),
            DynamicGuardSelector(
                [(PaceBall(bb), GotPace())]),
            DynamicGuardSelector([
                (TrackBall(bb), BallSeen()),
                # If the ball position is provided by teammate, look ahead
                (FindBall(bb), None)
            ]))
        super(PassBall, self).__init__(*args)

    def run(self):
        self.get_bb().set_state('assisting')
        super(PassBall, self).run()