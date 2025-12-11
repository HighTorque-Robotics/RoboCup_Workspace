from dbehavior.core import Skill
from dbehavior.util import VecPos, Timer
from dbehavior.util.mathutil import (deg2rad, rad2deg,
                                     radian_normalization, angle_normalization,
                                     get_magnitude, degree_between, sind, cosd)
from dbehavior.btree.core import Status
from math import sin, cos, fabs, atan2
import rospy
import numpy as np


class KickZone:
    def __init__(self):
        self.kick_x_min = 12
        self.kick_x_max = 22
        self.kick_y_tol = 4
        self.kick_y_offset = 8
        self.kick_theta_offset = 0
        self.kick_theta_tol = deg2rad(10)

    def wished_pos(self, right_foot):
        """
        Get wish kick pos with foot offset.

        :param right_foot: whether to use right foot or not
        :type right_foot: bool
        :return: wished position
        :rtype: (float, float, float)
        """

        target_x = (self.kick_x_min + self.kick_x_max) / 2
        foot_traget_y = -self.kick_y_offset if right_foot else self.kick_y_offset
        wished_dir = -self.kick_theta_offset if right_foot else self.kick_theta_offset

        return target_x, foot_traget_y, wished_dir

    @property
    def x_range(self):
        return self.kick_x_max - self.kick_x_min

    @property
    def y_range(self):
        return self.kick_y_tol

    @property
    def theta_tol(self):
        return self.kick_theta_tol

    def can_kick(self, ball_pos, robot_pos, kick_dir):
        kick_state = self.convert_world_state_to_kick_state(ball_pos, robot_pos, kick_dir)
        return self.can_kick_left(kick_state) or self.can_kick_right(kick_state)

    def can_kick_left(self, kick_state):
        """
        Ability to kick with left foot

        :param kick_state: kick state
        :type kick_state: VecPos
        :return: kicking validity
        :rtype: bool
        """
        # get explicit names
        ball_x = kick_state.x
        ball_y = kick_state.y
        theta = kick_state.z
        kick_err = angle_normalization(theta + self.kick_theta_offset)
        # check validity
        x_ok = self.kick_x_min < ball_x < self.kick_x_max
        y_ok = fabs(ball_y - self.kick_y_offset) < self.kick_y_tol
        theta_ok = fabs(kick_err) < self.kick_theta_tol

        return x_ok and y_ok and theta_ok

    def can_kick_right(self, kick_state):
        """
        Ability to kick with right foot

        :param kick_state: kick state
        :type kick_state: VecPos
        :return: kicking validity
        :rtype: bool
        """
        # get explicit names
        ball_x = kick_state.x
        ball_y = kick_state.y
        theta = deg2rad(kick_state.t)
        kick_err = radian_normalization(theta - self.kick_theta_offset)
        # check validity
        x_ok = self.kick_x_min < ball_x < self.kick_x_max
        y_ok = fabs(ball_y + self.kick_y_offset) < self.kick_y_tol
        theta_ok = fabs(kick_err) < self.kick_theta_tol

        return x_ok and y_ok and theta_ok

    @staticmethod
    def convert_world_state_to_kick_state(ball_global, robot_pos, kick_dir):
        """
        Convert world state to kick state.

        :param ball_global: ball position
        :type ball_global: VecPos
        :param robot_pos: robot position
        :type robot_pos: VecPos
        :param kick_dir: kick direction
        :type kick_dir: float
        :return: kick state
        :rtype: VecPos
        """
        dx = ball_global.x - robot_pos.x
        dy = ball_global.y - robot_pos.y
        return VecPos(dx * cosd(robot_pos.z) + dy * sind(robot_pos.z),
                      -dx * sind(robot_pos.z) + dy * cosd(robot_pos.z),
                      kick_dir - robot_pos.z)


class NeoApproach(Skill):

    def __init__(self, bb):
        super(NeoApproach, self).__init__(bb)
        self.state = 'far'
        self.far_aligner_p = 0.4
        self.rotate_aligner_p = 0.4
        self.near_aligner_p = 0.4
        self.rotate_lateral_p = 0.4
        self.near_lateral_p = 0.4
        self.setp_p = 1
        self.radius = 45

        self.shoot_interval = 3
        self.centering_corner = 2.50
        self.elapsed_last_shoot = Timer(left_shift=1e6)
        self.time_since_near = Timer(left_shift=3600)

        self.idiot_mode = False  # attack with only the magnetometer
        self.lateral_kick_threshold = 75  # the range where we will do lateral kick
        self.enable_lateral_kick = False  # enable the lateral kick
        self.placement_margin = 30  # distance to target for entering rotate state

        self.expected_kick = 'classic'
        self.kick_right = True

        self.kick_zone = KickZone()

    def on_start(self):
        super(NeoApproach, self).on_start()
        self.state = 'far'
        self.elapsed_last_shoot = Timer(left_shift=1e6)
        self.time_since_near = Timer(left_shift=3600)

    def execute(self):
        print(self.state)

        goal_azimuth = angle_normalization(self.get_kick_cap())
        ball_field = self.bb.vision_info.ball_field
        ball_azimuth = angle_normalization(ball_field.z)
        ball_distance = get_magnitude(ball_field)
        attack_azimuth = angle_normalization(self.get_kick_cap())

        lateral_allowed = self.enable_lateral_kick
        if self.state != 'near':
            if lateral_allowed and fabs(goal_azimuth) > self.lateral_kick_threshold:
                self.expected_kick = 'lateral'
                self.kick_right = goal_azimuth > 0
            else:
                self.expected_kick = 'classic'
                self.kick_right = True

        target_x, foot_target_y, wished_dir = self.kick_zone.wished_pos(self.kick_right)
        x_range = self.kick_zone.x_range
        y_range = self.kick_zone.y_range
        target_azimuth = angle_normalization(self.get_kick_cap()) + wished_dir

        good_align_target_center = fabs(target_azimuth < 15)
        good_align_target_left = target_azimuth > 8
        good_align_target_right = target_azimuth < -8
        good_align_target = good_align_target_center
        if not self.idiot_mode:
            good_align_target = good_align_target_center or good_align_target_left or good_align_target_right

        good_align_ball = fabs(ball_azimuth) < 15

        if self.state == 'far':
            big_radius = 2.0 * self.radius

            B = VecPos(ball_field.x, ball_field.y)
            V = VecPos(big_radius * cosd(attack_azimuth + 180), big_radius * sind(attack_azimuth + 180))
            T = B + V
            IA = B + V.rotate(80)
            IB = B + V.rotate(-80)
            theta = angle_normalization(ball_azimuth + 180 - attack_azimuth)
            target = T

            if fabs(theta) < 75:
                if get_magnitude(IA) < get_magnitude(IB):
                    print('IA 1')
                    target = IA
                else:
                    print('IB 1')
                    target = IB
            else:
                if self.enable_lateral_kick:
                    if get_magnitude(IA) < get_magnitude(target):
                        print('IA 2')
                        target = IA
                    if get_magnitude(IB) < get_magnitude(target):
                        print('IB 2')
                        target = IB

                    a = degree_between(target, B)

                    if get_magnitude(target) < self.placement_margin or a > 60:
                        rospy.loginfo('FAR: placement reached, going to near')
                        self.state = 'near'

            dest = self.field2world(target)
            dest.z = attack_azimuth
            self.bb.behavior_info.dest = dest.to_vector3()
            self.bb.behavior_info.final_dest = dest.to_vector3()

            error_azimuth = atan2(target.y, target.x)
            step_cos = cos(error_azimuth)
            if step_cos < 0:
                step_cos = 0

            print('({},{},{})'.format(step_cos * target.x, 0, rad2deg(error_azimuth) * self.far_aligner_p))
            self.walk(step_cos * target.x * self.setp_p,
                      0,
                      rad2deg(error_azimuth) * self.far_aligner_p)
            if ball_distance < 1.2 * self.radius:
                rospy.loginfo('FAR: Going to rotate, we are close enough')
                self.state = 'rotate'
            # if step_cos * target.x * self.setp_p < 1e-1 and ball_distance < big_radius:
            #     self.state = 'rotate'
            #     return Status.RUNNING

        elif self.state == 'rotate':
            self.time_since_near.restart()
            print('({},{},{})'.format(ball_distance - self.radius,
                                      target_azimuth * self.rotate_lateral_p,
                                      ball_azimuth * self.rotate_aligner_p))
            self.walk(ball_distance - self.radius,
                      target_azimuth * self.rotate_lateral_p,
                      ball_azimuth)
            if ball_distance > 2.5 * self.radius:
                rospy.loginfo('ROTATE: Ball is too far, going to far')
                self.state = 'far'
            elif ball_distance < 1.2 * self.radius and good_align_target and good_align_ball:
                rospy.loginfo('ROTATE: Good alignment, going to near')
                self.state = 'near'

        elif self.state == 'near':
            self.time_since_near.restart()
            turn = 0
            if not good_align_target:
                turn = self.rotate_aligner_p / 5 * target_azimuth
            else:
                turn = 0

            err_x = ball_field.x - target_x
            err_y = ball_field.y - foot_target_y
            forward_err_x = err_x

            if fabs(err_y) > 12.5:
                forward_err_x = 0

            self.walk(forward_err_x,
                      self.near_lateral_p * err_y,
                      turn)

            if fabs(ball_field.y) > self.radius:
                rospy.loginfo('NEAR: Ball is too aside, going to rotate')
                self.state = 'rotate'
            elif ball_distance > 1.8 * self.radius:
                rospy.loginfo('NEAR: Ball is too far, going to far')
                self.state = 'far'

        return Status.RUNNING

    def get_kick_cap(self):
        robot_pos = VecPos.from_vector3(self.bb.vision_info.robot_pos)
        ball_global = VecPos.from_vector3(self.bb.vision_info.ball_global)
        attack_target = self.bb.attack_target
        return (attack_target - ball_global).z - robot_pos.z

    def field2world(self, pos):
        tmp = pos
        robot_pos = self.bb.vision_info.robot_pos
        tmp.rotate(-robot_pos.z)
        tmp += robot_pos
        return tmp
