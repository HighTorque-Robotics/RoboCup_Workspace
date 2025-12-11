from dbehavior.core import Skill
from dbehavior.btree.core import Status
from dbehavior.btree.branch import Parallel, Selector
from dbehavior.util import (AttackPointSolver, WalkToPointSolver, VecPos, Timer)
from dbehavior.util.mathutil import (angle_normalization, radians, degrees,
                                     atan2, get_dis, abs_angle_diff,
                                     get_magnitude, get_angle)
from dbehavior.skill import TrackBall

from math import sin, cos, fabs
import rospy


class WalkTowardsBall(Skill):
    """
    Walk towards the ball.
    """

    def __init__(self, bb):
        super(WalkTowardsBall, self).__init__(bb)
        self.attack_solver = AttackPointSolver()
        self.walk_solver = WalkToPointSolver(bb.param)

        self.DIS_TURNING_AROUND = 80
        self.TARGET_BALL_DIS = 20
        self.BALL_Y_MAX = 8
        self.TARGET_ANGLE_TOL = 10
        self.TARGET_DIS_TOL = 15

    def _got_dest_simple(self, dst):
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
        ball_field = self.bb.vision_info.ball_field

        dis = get_dis(dst, robot_pos)
        diff_angle = abs_angle_diff(dst.z - robot_pos.z)

        return dis < self.TARGET_DIS_TOL and \
               fabs(ball_field.y) < self.BALL_Y_MAX and \
               diff_angle < self.TARGET_ANGLE_TOL and \
               get_magnitude(ball_field) < self.TARGET_BALL_DIS

    def execute(self):
        final_dest, dest, rub = self.attack_solver.calc_attack_point(self.bb)

        # succeed when robot has got to ball
        if self._got_dest_simple(final_dest):
            self.step()
            rospy.logfatal('dest:{}'.format(final_dest))
            rospy.logdebug('[WalkTowardsBall] succeed cuz got dest')
            return Status.SUCCEEDED

        # failed when ball is behind robot
        elif -30 < rub.x < 0 and abs(rub.y) < 30:
            self.step()
            rospy.logdebug('[WalkTowardsBall] failed cuz ball is behind robot')
            return Status.FAILED

        else:
            robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
            ball_field = VecPos.from_vector3(self.bb.vision_info.ball_field)
            vx = self.bb.motion_info.robotCtrl.x

            # directly go to final destination when close to the current destination
            if dest is not final_dest:
                dest_field = VecPos.calc_field_position(final_dest, robot_pos)
                if dest_field.x < -20 or self._got_dest(dest):
                    rospy.logdebug('[WalkTowardsBall] close to final dest')
                    dest = final_dest

            # feed final and current destination to BehaviorInfo
            self.bb.behavior_info.dest = dest.to_vector3()
            self.bb.behavior_info.final_dest = final_dest.to_vector3()

            # solve walk action
            x, y, t = self.walk_solver.solve_global(dest, robot_pos, vx)
            # angle = degree_between(ball_global, robot_pos)
            # diff = angle_normalization(self.bb.field_angle - angle + 180)

            # turn minor angle when walking to ball from far away
            if self.DIS_TURNING_AROUND < ball_field.length():
                if rub.x < 10:  # and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 3
                    else:
                        t += 3

                elif rub.x > 10:  # and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 3
                    else:
                        t += 3

            self.walk(x, y, t)
            # rospy.logdebug('[WalkTowardsBall] walk to ({:2.2f},{:2.2f},{:2.2f})'.format(x, y, t))
            # self.crouch()
            return Status.RUNNING


class WalkBehindBall(Skill):
    """
    Walk behind the ball.
    """

    def __init__(self, bb):
        super(WalkBehindBall, self).__init__(bb)

        self.attack_solver = AttackPointSolver()
        self.walk_solver = WalkToPointSolver(bb.param)
        self.stop_timer = Timer()
        self.stop_timer_inited = False

        self.SAFE_DIST = 20
        self.ANGLE_PER_TICK = 20
        self.CLOCK_WISE = -1
        self.ANTI_CLOCK_WISE = 1

        self.TARGET_BALL_DIS = 20
        self.BALL_Y_MAX = 8
        self.TARGET_ANGLE_TOL = 10
        self.TARGET_DIS_TOL = 15

        self.stop_score = 0
        self.STOP_GAIN = 2
        self.NOT_STOP_GAIN = 0.5

        self.BACK_DIST = 100
        self.BACK_X_MAX = 15
        self.BACK_X_MIN = -100

    def on_start(self):
        self.stop_timer.restart()
        self.stop_score = 0

    def _got_dest_simple(self, dst):
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
        ball_field = self.bb.vision_info.ball_field

        dis = get_dis(dst, robot_pos)
        diff_angle = abs_angle_diff(dst.z - robot_pos.z)

        return dis < self.TARGET_DIS_TOL and diff_angle < self.TARGET_ANGLE_TOL

    def execute(self):
        self.look_at(45, 0)

        ball_field = VecPos.from_vector3(self.bb.vision_info.ball_field)
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
        vx = VecPos.from_vector3(self.bb.motion_info.robotCtrl).x
        final_dest, _, rub = self.attack_solver.calc_attack_point(self.bb)

        # if ball_field.length() <= self.BACK_DIST and \
        # if self.BACK_X_MIN <= rub.x <= self.BACK_X_MAX:
        #     self.walk(-2, 0, 0)
        #     return Status.RUNNING

        if ball_field.length() <= self.SAFE_DIST + 5 and ball_field.x >= 0:
            if fabs(ball_field.y) < self.BALL_Y_MAX and abs(robot_pos.z -
                                                            final_dest.z) < 10:
                self.stop_score += self.stop_timer.elapsed() * self.STOP_GAIN
            else:
                self.stop_score -=  \
                    self.stop_timer.elapsed() * self.NOT_STOP_GAIN
            self.stop_timer.restart()
            rospy.logdebug('walking stop score: {}'.format(self.stop_score))

            if self.stop_score >= 1.0:
                rospy.logwarn('[WalkBehindBall] succeed cuz ball is near')
                self.step()
                return Status.SUCCEEDED
            elif self.stop_score <= -5:
                self.step()
                return Status.FAILED

        # if self.stop_timer_inited and self.stop_timer.elapsed() >= 5:
        #     rospy.logdebug('[WalkBehindBall] succeed cuz ball is near')
        #     self.step()
        #     return Status.SUCCEEDED

        # succeed when robot has got to ball
        if self._got_dest_simple(final_dest):
            rospy.logfatal('dest:{}'.format(final_dest))
            rospy.logwarn('[WalkBehindBall] succeed cuz got dest')
            self.step()
            return Status.SUCCEEDED

        if rub.y > 0:
            direction = self.CLOCK_WISE
        else:
            direction = self.ANTI_CLOCK_WISE

        r = self.SAFE_DIST
        theta = self.ANGLE_PER_TICK * direction

        beta = angle_normalization(180.0 -
                                   degrees(atan2(ball_field.y, ball_field.x)))
        alpha = radians(theta - beta)
        r_vec = VecPos(r * cos(alpha), r * sin(alpha))

        dest = ball_field + r_vec
        dest.z = angle_normalization(alpha + 180)

        self.bb.behavior_info.dest = dest.to_vector3()
        self.bb.behavior_info.final_dest = final_dest.to_vector3()

        x, y, t = self.walk_solver.solve_field(dest, vx)
        # t *= 0.5
        # y *= 0.5
        if abs(x) < 1.5 * self.SAFE_DIST:
            x *= 0.5

        self.walk(x, y, t)
        # rospy.logdebug('[WalkBehindBall] walk to ({:2.2f},{:2.2f},{:2.2f})'.format(x, y, t))
        return Status.RUNNING


class WalkStraightTowardsBall(Skill):
    pass


class ApproachBall(Parallel):

    def __init__(self, bb):
        args = (Selector(WalkTowardsBall(bb),
                         WalkBehindBall(bb)), TrackBall(bb))
        super(ApproachBall, self).__init__(*args)

    def run(self):
        self.get_bb().set_state('placing')
        super(ApproachBall, self).run()
