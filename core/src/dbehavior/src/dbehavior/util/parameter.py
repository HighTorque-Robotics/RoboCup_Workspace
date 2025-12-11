import rospy
from dbehavior.util import VecPos


class Parameter:
    """
    All parameters from ROS param sever.
    """

    def __init__(self):
        self.update()

    def update(self):
        # global parameters
        self.get_param('robot_id', '~RobotId')
        self.get_param('attack_right', '/ZJUDancer/AttackRight')
        self.get_param('use_dribble', '/ZJUDancer/AlongGrass')
        self.get_param('simulation', '/ZJUDancer/Simulation')
        self.get_param('team_number', '/ZJUDancer/TeamNumber')

        # behavior parameters
        self.get_param('role', '~role')
        self.get_param('print_routes', '~dbehavior/debug/print_routes')
        self.get_param('print_status', '~dbehavior/debug/print_status')

        setattr(self, 'motion_init_time', 3.0)
        # # motion parameters
        # self.get_param(
        #     'motion_init_time',
        #     '/dmotion_{}/dmotion/state/imu_prepare_time'.format(self.robot_id),
        #     '/dmotion_{}/dmotion/hardware/imu_prepare_time'.format(
        #         self.robot_id))

        # Constants
        # robot
        self.get_param('num_player', '~dbehavior/robot/num_player')
        self.get_param('max_pitch', '~dbehavior/robot/max_pitch')
        self.get_param('min_pitch', '~dbehavior/robot/min_pitch')
        self.get_param('max_yaw', '~dbehavior/robot/max_yaw')
        self.get_param('walk_speed', '~dbehavior/robot/walk_speed')
        self.get_param('turn_speed', '~dbehavior/robot/turn_speed')
        # field geometry in cm, refer to RoboCup Humanoid Rules
        self.get_param('field_length', '~dbehavior/geometry/field_length')
        self.get_param('field_width', '~dbehavior/geometry/field_width')
        self.get_param('goal_depth', '~dbehavior/geometry/goal_depth')
        self.get_param('goal_width', '~dbehavior/geometry/goal_width')
        self.get_param('goal_height', '~dbehavior/geometry/goal_height')
        self.get_param('goal_area_length',
                       '~dbehavior/geometry/goal_area_length')
        self.get_param('goal_area_width', '~dbehavior/geometry/goal_area_width')
        self.get_param('penalty_mark_distance',
                       '~dbehavior/geometry/penalty_mark_distance')
        self.get_param('center_circle_diameter',
                       '~dbehavior/geometry/center_circle_diameter')
        self.get_param('border_strip_width',
                       '~dbehavior/geometry/border_strip_width')
        self.get_param('line_width', '~dbehavior/geometry/line_width')
        self.whole_width = self.field_length + self.border_strip_width * 2
        self.whole_height = self.field_width + self.border_strip_width * 2
        # team info
        self.get_param('teaminfo_outdated', '~dbehavior/team/teaminfo_outdated')
        self.get_param('ball_share_enabled',
                       '~dbehavior/team/ball_share_enabled')
        self.get_param('pace','~dbehavior/team/pace')

        # Default position
        self.pos_role = self.role.lower()
        self.pos_role = self.pos_role.replace('gc', '').replace('fake',
                                                                '').replace(
                                                                    'test', '')
        if (self.pos_role not in [
                'striker', 'defender', 'supporter', 'goalkeeper'
        ]):
            self.pos_role = 'others'
        self.get_vecpos('start_pos',
                        '~dbehavior/role/{}/start_pos'.format(self.pos_role))
        self.get_vecpos('init_pos',
                        '~dbehavior/role/{}/init_pos'.format(self.pos_role))
        self.get_vecpos('patrol_pos_a',
                        '~dbehavior/role/{}/patrol_pos_a'.format(self.pos_role))
        self.get_vecpos('patrol_pos_b',
                        '~dbehavior/role/{}/patrol_pos_b'.format(self.pos_role))
        self.get_param('team_radius',
                       '~dbehavior/role/{}/team_radius'.format(self.pos_role))
        if self.pos_role in ['striker', 'supporter']:
            self.get_vecpos(
                'kickoff_pos',
                '~dbehavior/role/{}/kickoff_pos'.format(self.pos_role))
        elif self.pos_role == 'defender':
            self.get_param(
                'defend_radius',
                '~dbehavior/role/{}/defend_radius'.format(self.pos_role))

    def get_param(self, name, path, bak_path=None):
        try:
            setattr(self, name, rospy.get_param(path))
        except KeyError:
            rospy.logerr('Can not get param {} from {}'.format(name, path))
            if bak_path is not None:
                self.get_param(name, bak_path)

    def get_vecpos(self, name, path):
        try:
            vec = rospy.get_param(path)
            if len(vec) not in [2, 3]:
                rospy.logerr('Can not get convert param {} to VecPos'.format(
                    name, path))
            setattr(self, name, VecPos.from_list(vec))
        except KeyError:
            rospy.logerr('Can not get param {} from {}'.format(name, path))
