from dbehavior.core import Skill
from dbehavior.btree.core import Status
from dbehavior.util import AttackPointSolver, VecPos, Timer
import rospy


class DribbleConfig(object):
    """
    Configuration for ``Dribble`` skill.
    """

    def __init__(self):
        self.DRIBBLE_THRES = 30
        self.EPSO = 1e-10

        self.DRIBBLE_SAFE = -20
        self.DRIBBLE_X_B = -2
        self.DRIBBLE_X_AD_B = -3  # 2
        self.DRIBBLE_X_AD_F = 3  # 2
        self.DRIBBLE_X_MID = 3
        self.DRIBBLE_X_TOP = 4

        self.DRIBBLE_Y_B = 1
        self.DRIBBLE_Y_AD_B = 2
        self.DRIBBLE_Y_AD_F = 2
        self.DRIBBLE_Y_MID = 1
        self.DRIBBLE_Y_TOP = self.EPSO

        self.STEP_L = 12
        self.STEP_R = -12
        self.RM_L = 3.5
        self.RM_R = -15.5
        self.LM_L = 15.5
        self.LM_R = -3.5

        self.x_can = [self.DRIBBLE_X_B, self.DRIBBLE_X_B, self.DRIBBLE_X_AD_B, 0, self.DRIBBLE_X_AD_F,
                      self.DRIBBLE_X_MID, self.DRIBBLE_X_TOP,
                      self.DRIBBLE_X_TOP]
        self.y_can = [self.EPSO, self.DRIBBLE_Y_B, self.DRIBBLE_Y_B, self.DRIBBLE_Y_AD_B, self.DRIBBLE_Y_AD_F,
                      self.DRIBBLE_Y_MID, self.DRIBBLE_Y_TOP,
                      self.EPSO]
        self.rec_can = [self.x_can[i] * 1.0 / self.y_can[i] for i in range(0, len(self.x_can))]


class Dribble(Skill):
    """
    Dribble ball forward.
    """

    def __init__(self, bb):
        super(Dribble, self).__init__(bb)
        self.cfg = DribbleConfig()
        self.attack_solver = AttackPointSolver()

        self.sx, self.sy = 0, 0
        self.exit_cycle = 0
        self.exit_cycle2 = 0
        self.exit_cycle3 = 0
        self.exit_timer3 = None
        self.exit_cycle4 = 0
        self.prev_angle = 0
        self.started = False

    def on_start(self):
        self.sx, self.sy = 0, 0
        self.exit_cycle = 0
        self.exit_cycle2 = 0
        self.exit_cycle3 = 0
        self.exit_timer3 = None
        self.exit_cycle4 = 0
        self.prev_angle = 0
        self.started = False

    def execute(self):
        # exit when kicking is enabled
        if self.bb.kick_enabled:
            self.step()
            rospy.logdebug('[Dribble] failed cuz kicking is enabled')
            return Status.FAILED

        # Exit when the ball is too far away from feet
        ball_field = VecPos.from_vector3(self.bb.vision_info.ball_field)
        # rospy.logdebug('[Dribble] ball ({},{})'.format(ball_field.x, ball_field.y))
        if ball_field.x < 15 and abs(ball_field.y) > 20:
            self.exit_cycle4 += 1
        if self.exit_cycle4 > 3:
            self.step()
            rospy.logdebug('[Dribble] failed cuz the ball is far away from feet in y axis')
            return Status.FAILED

        # Exit when too many tries for adjusting position
        final_dest, dest, rub = self.attack_solver.calc_attack_point(self.bb)
        if not self._got_dest_loose(final_dest):
            self.step()
            self.exit_cycle3 += 1
            self.exit_timer3 = Timer()
        if self.exit_timer3 is not None and self.exit_timer3.elapsed() >= 0.3:
            self.step()
            rospy.logdebug('[Dribble] failed cuz too many tries for adjusting position')
            return Status.FAILED

        # Exit when ball is far away, and robot needs to walk forward it
        if ball_field.x > 40:
            self.step()
            rospy.logdebug('[Dribble] failed cuz the ball is far away in x axis')
            return Status.FAILED

        self.look_at(45, 0)
        vy = self.bb.motion_info.robotCtrl.y

        if vy <= 0:
            current_l = self.cfg.STEP_L - (self.cfg.STEP_L - self.cfg.RM_L) / 1.8 * abs(vy)
            current_r = self.cfg.STEP_R - (self.cfg.STEP_R - self.cfg.RM_R) / 1.8 * abs(vy)
        else:
            current_l = self.cfg.STEP_L - (self.cfg.STEP_L - self.cfg.LM_L) / 1.8 * abs(vy)
            current_r = self.cfg.STEP_R - (self.cfg.STEP_R - self.cfg.LM_R) / 1.8 * abs(vy)

        eye_y = (current_l + current_r) / 2.0
        diff_y = ball_field.y - eye_y
        # rospy.logdebug('[Dribble] eye_y: {}; ball_field_y: {}; diff_y: {}'
        #                .format(eye_y, ball_field.y, diff_y))

        if not (self.cfg.STEP_L * 1.5 > diff_y > self.cfg.STEP_R * 1.5):
            self.step()
            rospy.logdebug('[Dribble] failed cuz ball is far away from center of FOV')
            return Status.FAILED

        diff_x = ball_field.x + 10
        theta = diff_x / abs(diff_y + 0.00001)
        # rospy.logdebug('[Dribble] diff_x: {}; theta: {}'.format(diff_x, theta))

        for i in range(0, 7):
            if self.cfg.rec_can[i] <= theta <= self.cfg.rec_can[i + 1]:
                a_s = (theta * self.cfg.y_can[i] - self.cfg.x_can[i]) / (
                        self.cfg.x_can[i + 1] - self.cfg.x_can[i] - theta * (
                        self.cfg.y_can[i + 1] - self.cfg.y_can[i]))
                self.sx = self.cfg.x_can[i] + a_s * (self.cfg.x_can[i + 1] - self.cfg.x_can[i])
                self.sy = self.cfg.y_can[i] + a_s * (self.cfg.y_can[i + 1] - self.cfg.y_can[i])
            if diff_x >= 0:
                self.sx = min(self.sx, diff_x)
            else:
                self.sx = max(self.sx, diff_x)
            self.sy = min(abs(diff_y), self.sy)
        if diff_y < 0:
            self.sy = -self.sy
        t = 0

        self.walk(self.sx, self.sy, t)
        # rospy.logdebug('[Dribble] walk to ({:2.2f},{:2.2f},{:2.2f})'.format(self.sx, self.sy, t))
        return Status.RUNNING
