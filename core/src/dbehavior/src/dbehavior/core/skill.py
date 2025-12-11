import rospy
from dbehavior.btree.core import Leaf, Status
from dbehavior.util import ActionGenerator, VecPos
from dbehavior.util.mathutil import get_dis, abs_angle_diff
from math import atan2, radians, fabs


class Skill(Leaf):
    """
    Skill node to execute special skill.
    """

    def __init__(self, bb):
        """
        Initialize skill.

        :param bb: blackboard from behavior tree
        :type bb: DBlackboard
        """
        super(Skill, self).__init__()
        self.action_generator = ActionGenerator()

    @property
    def bb(self):
        """
        Returns the dblackboard of this task.

        :return: the blackboard
        :rtype: DBlackboard
        """
        return super(Skill, self).get_bb()

    def do(self, cmd):
        """
        Do given body command.

        :param cmd: given body command
        :type cmd: BodyCommand
        """
        self.bb.action_cmd.bodyCmd = cmd

    def crouch(self):
        """
        Let robot crouch.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.crouch()

    def step(self):
        """
        Let robot step at original place.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.walk(0, 0, 0)

    def turn(self, st):
        """
        Let robot turn around with given angle.

        :param st: angle (degree) to turn around
        :type st: float
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.turn(st)

    def look_at(self, pitch=0, yaw=0, pitch_speed=1, yaw_speed=1):
        """
        Let robot look at somewhere by adjusting camera pose.

        :param pitch: desired pitch value (in degree) of camera pose
        :type pitch: float
        :param yaw: desired yaw value (in degree) of camera pose
        :type yaw: float
        :param pitch_speed: rotation speed for adjusting pitch value
        :type pitch_speed: float
        :param yaw_speed: rotation speed for adjusting yaw value
        :type yaw_speed: float
        """
        pitch_speed=0.7
        yaw_speed=0.7
        self.bb.action_cmd.headCmd = self.action_generator.head(
            pitch, yaw, pitch_speed, yaw_speed)

    def kick(self, kick_type='normal', kick_side='left'):
        """
        Kick ball

        :param kick_type: kick type
        :type kick_type: str
        :param kick_side: kick side
        :type kick_side: str
        """
        if kick_type == 'normal' and kick_side == 'left':
            self.kick_left()
        if kick_type == 'normal' and kick_side == 'right':
            self.kick_right()
        if kick_type == 'fast' and kick_side == 'left':
            self.fast_kick_left()
        if kick_type == 'fast' and kick_side == 'right':
            self.fast_kick_right()
        if kick_type == 'side' and kick_side == 'left':
            self.side_kick_left()
        if kick_type == 'side' and kick_side == 'right':
            self.side_kick_right()

    def goalie_left(self):
        """
        Let goalie robot defending ball coming from left.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.goalie_left()

    def goalie_mid(self):
        """
        Let goalie robot defending ball coming from forward.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.goalie_mid()

    def goalie_right(self):
        """
        Let goalie robot defending ball coming from right.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.goalie_right()

    def backward(self):
        """
        Let robot go backward.
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.backward()

    def goto_global(self, dst):
        """
        Let robot go to a position in robot coordinate.
        :param dst: destination position in robot coordinate
        :type dst: VecPos
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.walk_pos(dst.x, dst.y, dst.z)
        self.bb.behavior_info.dest = dst


    def goto_field(self, dis):
        """
        Let robot go to a position in robot coordinate.
        :param dst: destination position in robot coordinate
        :type dst: VecPos
        """
        dst = VecPos()
        robot_pos = VecPos.from_vector3(self.get_bb().vision_info.robot_pos)

        dst = dis + robot_pos
        self.bb.action_cmd.bodyCmd = self.action_generator.walk_pos(dst.x, dst.y, dst.z)
        self.bb.behavior_info.dest = dst
        
    def goto_ball(self, dst):
        """
        Let robot go to a position in global coordinate.
        :param dst: destination position in global coordinate
        :type dst: VecPos
        """
        self.bb.action_cmd.bodyCmd = self.action_generator.kick_ball(dst.x, dst.y, dst.z)
        self.bb.behavior_info.dest = dst

    # TODO move see_ball flag into guard
    def face_ball(self):
        """
        Turn around towards ball and gaze at it.
        """
        if self.bb.vision_info.see_ball:
            ball_field = self.bb.ball_field
            angle = atan2(ball_field.y, ball_field.x)

            if angle > radians(self.bb.param.face_ball_thresh):
                self.walk(0, 0, 10)
            elif angle < radians(-self.bb.param.face_ball_thresh):
                self.walk(0, 0, -10)
            else:
                self.crouch()

            self.gaze_ball()

    # TODO move see_ball flag into guard
    def gaze_ball(self):
        """
        Gaze at ball.
        """
        if self.bb.vision_info.see_ball:
            track = self.bb.vision_info.ballTrack
            cur_plat = self.bb.motion_info.curPlat
            pitch = fabs(
                track.pitch - cur_plat.pitch
            ) > self.bb.param.gaze_ball_thresh and track.pitch or cur_plat.pitch
            yaw = fabs(
                track.yaw - cur_plat.yaw
            ) > self.bb.param.gaze_ball_thresh and track.yaw or cur_plat.yaw

            self.look_at(pitch, yaw, 1, 1)

    def _turned_to_target(self, target, angle_tol=20):
        """
        Check whether or not robot has turned to target direction.

        :param target: target direction(degree).
        :type target: float
        :param angle_tol: tolerance of angle
        :return: whether or not robot has turned to target direction.
        :rtype: bool
        """
        robot_pos = self.bb.vision_info.robot_pos

        angle = robot_pos.z
        dangle = abs_angle_diff(target - angle)
        return dangle < angle_tol

    def _got_dest(self, dst):
        """
        Check whether or not robot has gotten to destination (normal condition).

        :param dst: destination position.
        :type dst: VecPos
        :return: whether or not robot has gotten to destination
        :rtype: bool
        """
        robot_pos = self.bb.vision_info.robot_pos
        ball_field = self.bb.vision_info.ball_field

        angle = robot_pos.z
        dangle = abs_angle_diff(dst.z - angle)

        diff_x = dst.x - robot_pos.x
        diff_y = dst.y - robot_pos.y

        if ball_field.y > 0:
            return abs(diff_x) < self.bb.param.walk_dest_region and \
                -self.bb.param.walk_dest_region < diff_y < self.bb.param.walk_dest_region / 3 and \
                abs(dangle) < self.bb.param.walk_dest_re_angle
        else:
            return abs(diff_x) < self.bb.param.walk_dest_region and \
                -self.bb.param.walk_dest_region / 3 < diff_y < self.bb.param.walk_dest_region and \
                abs(dangle) < self.bb.param.walk_dest_re_angle

    def _got_dest_simple(self,
                         dst,
                         dis_tol=20,
                         angle_tol=10,
                         x_tol=20,
                         y_tol=20):
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)

        dis = get_dis(dst, robot_pos)
        diff_angle = abs_angle_diff(dst.z - robot_pos.z)
        dx = abs(dst.x - robot_pos.x)
        dy = abs(dst.y - robot_pos.y)
        # if dis > dis_tol:
        #     print('dist exceed')
        # if diff_angle > angle_tol:
        #     print('angle exceed')
        # if dx > x_tol:
        #     print('x exceed')
        # if dy > y_tol:
        #     print('y exceed')

        return dis < dis_tol and diff_angle < angle_tol and dx < x_tol and dy < y_tol

    def head_got_dest(self, pitch, yaw):
        """
        Check whether or not robot head has gotten to destination.

        :param pitch: desired pitch value of head
        :type pitch: float or int
        :param yaw: desired yaw value of head
        :type yaw: float or int
        :return: whether or not robot head has gotten to destination
        :rtype: bool
        """
        thresh = 8
        cur_plat = self.bb.motion_info.curPlat
        print(f"head_got_dest: pitch {pitch}, yaw {yaw}, cur_pitch {cur_plat.pitch}, cur_yaw {cur_plat.yaw}")
        return fabs(pitch - cur_plat.pitch) < thresh and fabs(
            yaw - cur_plat.yaw) < thresh

    def running(self):
        if self.bb.param.print_routes:
            self.print_running_route('Running')
        super(Skill, self).running()

    def success(self):
        if self.bb.param.print_routes:
            self.print_running_route('Success')
        super(Skill, self).success()

    def failure(self):
        if self.bb.param.print_routes:
            self.print_running_route('Failure')
        super(Skill, self).failure()

    def print_running_route(self, status):
        running_route = '{}: {}'.format(status,
                                        self.get_tree().get_running_route())
        # self.bb.behavior_info.running_route += '{}\n'.format(running_route)
        rospy.loginfo(running_route)


class Do(Skill):
    """
    Do simple action command.
    """

    def __init__(self, bb, cmd='crouch', pitch=0, yaw=0):
        super(Do, self).__init__(bb)
        self.cmd = cmd
        self.pitch = pitch
        self.yaw = yaw

    def execute(self):
        if self.cmd == 'crouch':
            self.crouch()
        elif self.cmd == 'step':
            self.step()
        elif self.cmd == 'look':
            self.look_at(self.pitch, self.yaw)
        elif self.cmd == 'back':
            self.walk(-1, 0, 0)
        else:
            self.crouch()
        return Status.SUCCEEDED
