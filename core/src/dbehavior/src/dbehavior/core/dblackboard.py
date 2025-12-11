# -*- coding: utf-8 -*-
import rospy
import sys
sys.path.append("/home/nvidia/robocup_ws/lib/devel/lib/python3/dist-packages")

from dmsgs.msg import (MotionInfo, VisionInfo, ActionCommand, BehaviorInfo,
                       GCInfo, TeamInfo)
from dmsgs.srv import (ToggleAMCL, ResetParticlePoint, ResetParticleLeftTouch,
                       ResetParticleRightTouch, ResetParticleTouchLine,
                       SetInitOrientation)
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from dbehavior.btree.core import Blackboard
from dbehavior.util import (Parameter, ActionGenerator, AttackPointSolver,
                            VecPos, Timer)
from dbehavior.util.mathutil import Inf, get_angle, get_magnitude, mirror

from math import sqrt


class DBlackboard(Blackboard):
    """
    Special implementation of Blackboard class for humanoid project of ZJUDancer.
    """

    def __init__(self):
        super(DBlackboard, self).__init__()
        # determined actions
        self.action_cmd = ActionCommand()
        self.action_generator = ActionGenerator()
        self.timer_start = Timer()

        # information from other modules
        self.motion_info = MotionInfo()
        self.vision_info = VisionInfo()
        self.gc_info = GCInfo()
        self.behavior_info = BehaviorInfo()

        # flags and useful information
        self.attack_target = VecPos(550, 0)
        # ball memory
        self.ball_field_last = None
        self.ball_global_last = None
        self.timer_ball_lost = Timer()
        # lower board connection
        self.timer_lower_board_lost = Timer(left_shift=1)
        self.timer_lower_board_reconnected = Timer()
        # game controller
        self.gc_connection_inited = False
        self.timer_gc_lost = Timer(left_shift=9999)
        self.timer_gc_initial = Timer()
        self.timer_gc_ready = Timer()
        self.re_entry = False
        self.timer_re_entry = Timer()
        self.enemy_free_kick = False
        self.timer_gc_playing = Timer(left_shift=9999)
        self.timer_state2_normal = Timer(left_shift=9999)
        # kicking
        self.timer_kick_success = Timer(left_shift=9999)
        self.kicking = False
        self.target_used = True
        # goal keeper
        self.exit_goalie_attack = False

        # team
        self.team_ball_field = None
        self.team_ball_global = None
        self.last_closest_to_ball_timestamp = 9999
        self.assist_point = None
        self.assist_timer = Timer()
        self.my_priority = False

        # motion
        self.timer_motion_re_stable = Timer(9999)
        self.motion_stable_last = False
        self.falled = False
        # yaw memory
        self.yaw_correct_last = Timer()
        # circle memory
        self.circle_field_last = None
        self.timer_circle_lost = Timer()
        # goal memory
        self.goal_field_last = None
        self.timer_goal_lost = Timer()

        # subscriber and publisher for ROS topic
        self.param = Parameter()
        self.team_info = [None] * self.param.num_player
        self.id = self.param.robot_id
        
        if self.online_striker != 0:
            self.kicker_id = self.online_striker
        elif self.online_striker != 0:
            self.kicker_id = self.online_defender
        else:
            self.kicker_id = self.id
        self.mates_online = [False, False, False, False, False, False]
        rospy.Subscriber('/dmotion_{}/MotionInfo'.format(self.id), MotionInfo,
                         self.update_motion_info)
        rospy.Subscriber('/dvision_{}/VisionInfo'.format(self.id), VisionInfo,
                         self.update_vision_info)
        rospy.Subscriber('/dnetwork_{}/GCInfo'.format(self.id), GCInfo,
                         self.update_gc_info)
        rospy.Subscriber('/dnetwork_{}/TeamInfo'.format(self.id), TeamInfo,
                         self.update_team_info)
        rospy.Subscriber('/dnetwork_{}/Command'.format(self.id), ActionCommand,
                         self.update_command)
        rospy.Subscriber('/humanoid/ReloadBehaviorConfig', String,
                         self.reload_parameters)
        
        self.action_cmd_publisher = rospy.Publisher(
            '/dbehavior_{}/ActionCommand'.format(self.id),
            ActionCommand,
            queue_size=1)
        self.behavior_info_publisher = rospy.Publisher(
            '/dbehavior_{}/BehaviorInfo'.format(self.id),
            BehaviorInfo,
            queue_size=1)

    def print_information(self):
        rospy.logwarn('Role: {}'.format(self.param.role))
        rospy.logwarn('Start pos: {}'.format(self.start_pos))
        rospy.logwarn('Init pos: {}'.format(self.init_pos))
        rospy.logwarn(
            'Attack: {}'.format('right' if self.param.attack_right else 'left'))
        self.draw_field_art()

    def draw_field_art(self):

        def set_pos(field_art, role='A'):
            field_art = field_art.replace(role, '■')
            all_role = ['A', 'B', 'C', 'D']
            all_role.remove(role)
            for other_role in all_role:
                field_art = field_art.replace('─ {} ─'.format(other_role),
                                              '─────')
                field_art = field_art.replace(other_role, ' ')
            return field_art

        if self.param.attack_right:
            field_art = '''
 ┌─ D ──────────── A ───┬──────────────────────┐
 ├──┐                   │                   ┌──┤
 │  │                   │                   │  │
 │  │              A    │                   │  │
 │  │                   │                   │  │
 │  │                  .│.                  │  │
 │ D│      +       B (  │  )         +      │  │
 │  │                  .│.                  │  │
 │  │                   │                   │  │
 │  │      C            │    enemy field    │  │
 │  │                   │                   │  │
 ├──┘                   │                   └──┤
 └──────── C ───── B ───┴──────────────────────┘
 '''
        else:
            field_art = '''
 ┌──────────────────────┬─── A ──────────── D ─┐
 ├──┐                   │                   ┌──┤
 │  │                   │                   │  │
 │  │                   │    A              │  │
 │  │                   │                   │  │
 │  │                  .│.                  │  │
 │  │      +         (  │  ) B       +      │D │
 │  │                  .│.                  │  │
 │  │                   │                   │  │
 │  │   enemy field     │            C      │  │
 │  │                   │                   │  │
 ├──┘                   │                   └──┤
 └──────────────────────┴─── B ───── C ────────┘
 '''
        if self.param.pos_role == 'supporter':
            field_art = set_pos(field_art, 'A')
        elif self.param.pos_role == 'striker':
            field_art = set_pos(field_art, 'B')
        elif self.param.pos_role == 'defender':
            field_art = set_pos(field_art, 'C')
        elif self.param.pos_role == 'goalkeeper':
            field_art = set_pos(field_art, 'D')

        rospy.logwarn(field_art)

    def reset(self):
        """Reset action command for new step."""
        self.action_cmd = ActionCommand()
        self.action_cmd.bodyCmd = self.action_generator.crouch()
        self.behavior_info = BehaviorInfo()
        self.behavior_info.attack_right = self.param.attack_right
        self.behavior_info.current_role = self.current_role
        self.behavior_info.time_since_last_kick = self.timer_kick_success.elapsed()

        if not self.lower_board_lost:
            self.set_state('playing')
        else:
            self.set_state('inactive')

    def publish(self):
        print("11111111")
        self.behavior_info.attack_target = self.attack_target.to_vector3()
        self.behavior_info.team_play_priority = False
        self.check_team_info()
        self.behavior_info.mates_online = self.mates_online
        # print("\n\ngait type is:", self.action_cmd.bodyCmd.gait_type)
        self.action_cmd_publisher.publish(self.action_cmd)
        self.behavior_info_publisher.publish(self.behavior_info)

    def update_motion_info(self, msg):
        """
        Callback function for receiving motion info.
        :param msg: motion information
        :type msg: MotionInfo
        """
        self.motion_info = msg
        if self.lower_board_lost and self.motion_info.lower_board_connected:
            self.timer_lower_board_reconnected.restart()

            # if GCPlaying when lower board is re-connected, 
            # then it's the case of re-entry
            if (self.gc_info.connected and self.gc_info.state is GCInfo.STATE_PLAYING) or \
                    not self.gc_info.connected:
                # TODO maybe 'not self.param.gc_listened' is better
                self.re_entry = True
                self.timer_re_entry.restart()
            else:
                self.re_entry = False

        if self.motion_info.lower_board_connected or self.param.simulation:
            self.timer_lower_board_lost.restart()

        if not self.motion_info.stable:
            # Cancel ball handling state when fall down
            self.set_state('searching')
            self.timer_motion_re_stable.restart()

        if self.motion_info.status == MotionInfo.KICKING:
            self.kicking = True
        else:
            if self.kicking:
                self.timer_kick_success.restart()
            self.kicking = False

    def update_vision_info(self, msg):
        """
        Callback function for receiving vision info.
        :param msg: vision information
        :type msg: VisionInfo
        """
        self.vision_info = msg
        if self.vision_info.see_ball:
            self.ball_field_last = VecPos.from_vector3(
                self.vision_info.ball_field)
            self.ball_global_last = VecPos.from_vector3(
                self.vision_info.ball_global)
            self.timer_ball_lost.restart()

        if self.vision_info.see_circle:
            self.circle_field_last = VecPos.from_vector3(
                self.vision_info.circle_field)
            self.timer_circle_lost.restart()

        if self.vision_info.see_goal:
            self.goal_field_last = VecPos(self.vision_info.goal_field.x, self.vision_info.goal_field.y)
            self.timer_goal_lost.restart()
        
        if self.vision_info.correct_yaw:
            self.yaw_correct_last.restart()

    def announce_ball_handling(self):
        # TODO(daiz): Keep priority until kick or fall down?
        # if self.current_role is BehaviorInfo.ROLE_GOALIE and get_magnitude(self.ball_field) < 50:
        #     self.behavior_info.team_play_priority = TeamInfo.HIGH_PRIORITY
        # if self.current_role is BehaviorInfo.ROLE_GOALIE and get_magnitude(self.ball_field) < 70:
        #     self.behavior_info.team_play_priority = TeamInfo.NORMAL_PRIORITY
        # elif self.current_role is BehaviorInfo.ROLE_DEFENDER and get_magnitude(self.ball_field) < 100:
        #     self.behavior_info.team_play_priority = TeamInfo.NORMAL_PRIORITY
        # else:
        #     self.behavior_info.team_play_priority = TeamInfo.LOW_PRIORITY
        self.behavior_info.team_play_priority = self.my_priority
        # time_to_reach_ball = self.time_to_reach_ball
        # if time_to_reach_ball <= 5:
        #     self.behavior_info.team_play_priority = TeamInfo.HIGH_PRIORITY
        # elif 5 < time_to_reach_ball <= 10:
        #     self.behavior_info.team_play_priority = TeamInfo.NORMAL_PRIORITY
        # else:
        #     self.behavior_info.team_play_priority = TeamInfo.LOW_PRIORITY
        pass

    def update_gc_info(self, msg):
        """
        Callback function for receiving GC info.
        :param msg: GC information
        :type msg: GCInfo
        """
        if self.gc_info.state is not GCInfo.STATE_PLAYING and \
                msg.state is GCInfo.STATE_PLAYING:
            self.on_start = True
            self.start_timer = Timer()
        if self.gc_info.secondaryState is not GCInfo.STATE2_NORMAL and \
                msg.secondaryState is GCInfo.STATE2_NORMAL:
            self.timer_state2_normal = Timer()

        self.gc_info = msg
        if not self.gc_connection_inited:
            self.gc_connection_inited = True
        if msg.connected:
            self.timer_gc_lost.restart()
            if msg.state is GCInfo.STATE_INITIAL:
                self.timer_gc_initial.restart()
            if msg.state is GCInfo.STATE_READY:
                self.timer_gc_initial.restart()
            if msg.state is not GCInfo.STATE_PLAYING:
                self.timer_gc_playing.restart()

    def update_team_info(self, msg):
        """
        Callback function for receiving team info.
        :param msg: team information
        :type msg: TeamInfo
        """
        msg.recv_timestamp = rospy.Time.now()
        if msg.attack_right != self.param.attack_right:
            msg.dest = mirror(msg.dest)
            msg.final_dest = mirror(msg.final_dest)
            msg.attack_target = mirror(msg.attack_target)

        self.mates_online[msg.player_number - 1] = True
        self.team_info[msg.player_number - 1] = msg
        if msg.kicker_id != self.id and msg.kicker_id != 0:
            self.kicker_id = msg.kicker_id
            # print('--------distance_4-----------------')
            # print(self.get_team_info(self.id).ball_field)
            # print('--------distance_2-----------------')
            # print(self.get_team_info(self.kicker_id).ball_field)
            if True: #self.param.pos_role == 'striker': #self.closest_to_ball == self.id:
                ballfield1 = self.get_team_info(self.id).ball_field
                ballfield2 = self.get_team_info(self.kicker_id).ball_field
                if get_magnitude(ballfield1) < get_magnitude(ballfield2)- 50:
                    self.kicker_id = self.id
        if self.mates_online[self.kicker_id-1] == False or self.team_info[self.kicker_id - 1].see_ball == False :#and self.param.pos_role == 'striker' 
            self.kicker_id = self.id    
        self.behavior_info.kicker_id = self.kicker_id
        # rospy.logfatal("kicker_id:")
        # print(self.kicker_id)
        # rospy.logfatal("self_role:")
        # print(self.param.pos_role)
        # print(self.mates_online)

        
            

    def check_team_info(self):
        for i in range(1, 7):
            info = self.team_info[i-1]
            if info != None:
                elapsed = rospy.Time.now() - info.recv_timestamp
                if elapsed.to_sec() > self.param.teaminfo_outdated:
                    self.mates_online[i-1] = False
            else:
                self.mates_online[i-1] = False

    def update_command(self, msg):

        self.action_cmd = msg

    def update_joy(self, msg):
        """
        Callback function for receiving joy signal.
        :param msg: joy signal
        :type msg: Joy
        """
        self.joy_info = msg
        if self.joy_info.buttons[7]:  # 'start' button
            # print('start')
            self.timer_joystick_started.restart()
            self.joystick_started = True
        elif self.joy_info.buttons[6]:  # 'back' button
            # print('stop')
            self.joystick_started = False

    def reload_parameters(self, msg):
        self.param.update()

    def toggle_amcl(self):
        service_name = '/dvision_{}/toggle_amcl'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ToggleAMCL)
            func(True)
        except rospy.ServiceException as e:
            rospy.logerr('call toggle amcl error {}'.format(e))

    def reset_particle_touch_line(self, side):
        service_name = '/dvision_{}/reset_particles_touch_line'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticleTouchLine)
            func(side)
        except rospy.ServiceException as e:
            rospy.logerr('call reset particle left touch error {}'.format(e))

    def reset_particle_left_touch(self):
        service_name = '/dvision_{}/reset_particles_left_touch'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticleLeftTouch)
            func()
        except rospy.ServiceException as e:
            rospy.logerr('call reset particle left touch error {}'.format(e))

    def reset_particle_right_touch(self):
        service_name = '/dvision_{}/reset_particles_right_touch'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticleRightTouch)
            func()
        except rospy.ServiceException as e:
            rospy.logerr('call reset particles right touch error {}'.format(e))

    def reset_particle_point(self, point):
        service_name = '/dvision_{}/reset_particles_point'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticlePoint)
            func(point)
        except rospy.ServiceException as e:
            rospy.logerr('call reset particles point error {}'.format(e))

    def set_orientation(self, yaw):
        service_name = '/dvision_{}/reset_yaw'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, SetInitOrientation)
            func(yaw)
        except rospy.ServiceException as e:
            rospy.logerr('call SetOrientation error: {}'.format(e))

    @property
    def current_role(self):
        role = self.param.pos_role
        if role == 'striker':
            return BehaviorInfo.ROLE_STRIKER
        elif role == 'defender':
            return BehaviorInfo.ROLE_DEFENDER
        elif role == 'supporter':
            return BehaviorInfo.ROLE_SUPPORTER
        elif role == 'goalkeeper':
            return BehaviorInfo.ROLE_GOALIE
        else:
            return BehaviorInfo.ROLE_OTHER

    @property
    def start_pos(self):
        dest = self.param.start_pos

        if not self.param.attack_right:
            dest = dest.mirror_by_y_axis()

        return dest

    @property
    def init_pos(self):
        # if self.gc_info.kickoff and self.current_role is BehaviorInfo.ROLE_STRIKER:
        if self.gc_info.kickoff and self.current_role in \
         [BehaviorInfo.ROLE_STRIKER, BehaviorInfo.ROLE_SUPPORTER]:
            dest = self.param.kickoff_pos
        else:
            dest = self.param.init_pos

        if not self.param.attack_right:
            dest = dest.mirror()

        return dest

    @property
    def patrol_pos_a(self):
        dest = self.param.patrol_pos_a

        if not self.param.attack_right:
            dest = dest.mirror()

        return dest

    @property
    def patrol_pos_b(self):
        dest = self.param.patrol_pos_b

        if not self.param.attack_right:
            dest = dest.mirror_by_y_axis()

        return dest

    @property
    def lower_board_lost(self):
        return self.timer_lower_board_lost.elapsed() >= 1

    @property
    def lower_board_reconnected(self):
        return self.timer_lower_board_reconnected.elapsed(
        ) <= self.param.motion_init_time

    @property
    def ball_lost(self):
        return self.timer_ball_lost.elapsed() >= 5

    @property
    def see_ball(self):
        return self.vision_info.see_ball or self.team_ball_seen

    @property
    def ball_lost_recent(self):
        return self.timer_ball_lost.elapsed() <= 25

    @property
    def ball_field(self):
        if self.vision_info.see_ball:
            return self.vision_info.ball_field
        else:
            # I can't see ball but my teammate can
            return self.team_ball_field

    @property
    def ball_global(self):
        if self.vision_info.see_ball:
            return self.vision_info.ball_global
        else:
            return self.team_ball_global

    @property
    def team_ball_seen(self):
        if not self.param.ball_share_enabled:
            return False
        max_dis = 1082  # diagonal length of field
        max_score = 0
        best_id = 0
        for i in range(1, self.param.num_player + 1):
            info = self.get_team_info(i)
            if info is not None and info.see_ball:
                # if the teammate has low field quality, ignore
                if info.field_quality < 0.5:  # or info.field_consistency < 0.5:
                    continue
                dis = sqrt(info.ball_field.x * info.ball_field.x +
                             info.ball_field.y * info.ball_field.y)
                score = max_dis / dis
                if score > max_score:
                    max_score = score
                    best_id = i
        # print(best_id)
        if best_id == 0:
            return False
        info = self.get_team_info(best_id)
        self.team_ball_global = info.ball_global
        robot_pos = self.vision_info.robot_pos
        self.team_ball_field = VecPos.calc_field_position(
            self.team_ball_global, robot_pos)
        return True

    @property
    def mate_target(self):
        for robot_id in range(1, 7):
            if robot_id != self.id:
                t = self.get_team_info(robot_id)
                if t is not None:
                    if t.state is TeamInfo.BALL_HANDLING:
                        return t.attack_target
        return None

    @property
    def time_to_reach_ball(self):
        if self.vision_info.see_ball:
            robot_pos = VecPos.from_vector3(self.vision_info.robot_pos)
            ball_global = VecPos.from_vector3(self.vision_info.ball_global)
            rub = AttackPointSolver.calc_rub(ball_global, self.attack_target,
                                             robot_pos)

            angle = get_angle(rub)
            dis = get_magnitude(rub)

            return abs(angle/self.param.turn_speed) + dis / self.param.walk_speed
        else:
            return Inf

    def get_team_info(self, robot_id):
        info = self.team_info[robot_id - 1]
        if info is None:
            return None

        elapsed = rospy.Time.now() - info.recv_timestamp
        if elapsed.to_sec() > self.param.teaminfo_outdated:
            return None

        if info.incapacitated:
            return None

        # if info.team_number != self.param.team_number:
        #     return None

        return info

    @property
    def gc_lost(self):
        return not self.gc_info.connected and \
               self.timer_gc_lost.elapsed() >= self.param.teaminfo_outdated


    @property
    def online_mates(self):
        """
        list of online mates' id and role,
        with variable length.
        """
        mates = []
        for robot_id in range(1, 7):
            t = self.get_team_info(robot_id)
            if t is not None:
                mates.append((robot_id, t.role))

        return mates

    
    @property
    def online_striker(self):
        for t in self.online_mates:
            if t[1] == 1:
                return t[0]
        return 0  
    
    @property
    def online_strikers(self):
        strikers = []
        for t in self.online_mates:
            if t[1] == 1:
                strikers.append(t[0])
        return strikers
    
    @property
    def online_defender(self):
        for t in self.online_mates:
            if t[1] == 2:
                return t[0]
        return 0
 
    @property
    def closest_to_ball(self):
        distances = []
        
        # 遍历队员，收集距离信息
        for i in range(1, 7):
            info = self.get_team_info(i)
            if info is not None and info.see_ball:
                dis = get_magnitude(info.ball_field)
                distances.append((i, dis))     
        # 如果没有有效队员，返回 None 或其他默认值
        if not distances:
            return None
        # 找到最近队员的 ID
        best_id, shortest_dis = min(distances, key=lambda x: x[1])
        return best_id


    @property
    def mate_attacking(self):
        for robot_id in range(1, 7):
            if robot_id != self.id:
                t = self.get_team_info(robot_id)
                # FIXME(daiz): Use param instead of magic number
                # self.param.time_post_kick_tracking:
                if t is not None and t.time_since_last_kick < 1:
                    # t.state is TeamInfo.BALL_HANDLING:
                    return True
        return False

    @property
    def mate_ball_handling(self):
        if self.param.pace == True:
            return True
        else:
            return self.kicker_id != self.id #and self.param.pos_role == 'striker'
        # return True

    def set_state(self, state):
        if state == 'inactive':
            self.behavior_info.team_play_state = TeamInfo.INACTIVE
        elif state == 'initializing':
            self.behavior_info.team_play_state = TeamInfo.INITIALIZING
        elif state == 'playing':
            self.behavior_info.team_play_state = TeamInfo.PLAYING
        elif state == 'placing':
            self.behavior_info.team_play_state = TeamInfo.PLACING
        elif state == 'searching':
            self.behavior_info.team_play_state = TeamInfo.SEARCHING
        elif state == 'ball_handling':
            self.behavior_info.team_play_state = TeamInfo.BALL_HANDLING
        elif state == 'assisting':
            self.behavior_info.team_play_state = TeamInfo.ASSISTING

    @property
    def circle_lost(self):
        return self.timer_circle_lost.elapsed() >= 2

    @property
    def circle_seen(self):
        if not self.circle_field_last:
            return False
        elif self.vision_info.see_circle:
            return True
        elif not self.circle_lost:
            return True
        return False

    @property
    def goal_lost(self):
        return self.timer_goal_lost.elapsed() >= 2

    @property
    def goal_seen(self):
        if not self.goal_field_last:
            return False
        elif not self.goal_lost:
            return True
        return False
