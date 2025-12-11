from dbehavior.btree.core import Status
from dbehavior.btree.branch import Parallel
from dbehavior.core import Skill, Do
from dbehavior.skill import ScanField, FindBall, EntryScanField
from dbehavior.util import VecPos
from dmsgs.msg import GCInfo, BehaviorInfo
from math import fabs
from geometry_msgs.msg import Vector3
from dmsgs.srv import ResetParticleTouchLineRequest


class TrackCircle(Skill):
    """Track the center circle by rotating head and centering the circle in FOV."""

    def execute(self):
        if not self.bb.vision_info.see_circle and self.bb.circle_lost:
            return Status.FAILED
        if not self.bb.circle_field_last:
            return Status.FAILED
        circle = self.bb.circle_field_last
        if circle.x < 40 and fabs(circle.y) < 30:
            # if False:
            self.look_at(45, 0)
        else:
            track = self.bb.vision_info.circleTrack
            cur_plat = self.bb.motion_info.curPlat
            thresh = 5

            pitch = track.pitch if fabs(track.pitch -
                                        cur_plat.pitch) > thresh else cur_plat.pitch
            yaw = track.yaw if fabs(track.yaw -
                                    cur_plat.yaw) > thresh else cur_plat.yaw
            self.look_at(pitch, yaw, 1, 1)
        return Status.SUCCEEDED


class TrackGoal(Skill):
    """Track the goal center by rotating head and centering the circle in FOV."""

    def execute(self):
        if not self.bb.goal_field_last:
            return Status.FAILED
        goal = self.bb.goal_field_last
        track = self.bb.vision_info.goalTrack
        cur_plat = self.bb.motion_info.curPlat
        thresh = 5

        pitch = track.pitch if fabs(track.pitch -
                                    cur_plat.pitch) > thresh else cur_plat.pitch
        yaw = track.yaw if fabs(track.yaw -
                                cur_plat.yaw) > thresh else cur_plat.yaw
        self.look_at(pitch, yaw, 1, 1)
        return Status.SUCCEEDED


class PrepareIMU(Parallel):
    def __init__(self, bb):
        args = (
            Do(bb, cmd='crouch'),
            EntryScanField(bb)
        )
        super(PrepareIMU, self).__init__(*args)

    def run(self):
        # print("imuimuimu")
        self.get_bb().set_state('initializing')
        super(PrepareIMU, self).run()


class InitialParticles(Skill):
    def execute(self):
        # Get gc status
        gc_connected = self.bb.gc_info.connected
        gc_normal_set = self.bb.gc_info.state == GCInfo.STATE_SET and \
                        self.bb.gc_info.secondaryState is GCInfo.STATE2_NORMAL

        # Directly set position in SET status
        # Place striker manually when kick off
        # if gc_connected and gc_normal_set and self.bb.gc_info.kickoff and \
        #     self.bb.current_role is BehaviorInfo.ROLE_STRIKER:
        #     self.bb.reset_particle_point(self.bb.init_pos.to_vector3())
        # elif gc_connected and gc_normal_set and self.bb.current_role is BehaviorInfo.ROLE_GOALIE:
        #     self.bb.reset_particle_point(self.bb.init_pos.to_vector3())

        # Get into the field from the touch line
        if self.bb.param.pos_role in ['supporter', 'defender']:
        # if self.bb.param.pos_role in ['supporter', 'defender']:
            # Set orientation
            self.bb.set_orientation(self.bb.start_pos.z)
            if self.bb.param.attack_right:
                if self.bb.start_pos.y > 0:
                    self.bb.reset_particle_touch_line(
                        ResetParticleTouchLineRequest.TOUCH_LINE_LEFT_TOP)
                else:
                    self.bb.reset_particle_touch_line(
                        ResetParticleTouchLineRequest.TOUCH_LINE_LEFT_BOTTOM)
            else:
                if self.bb.start_pos.y > 0:
                    self.bb.reset_particle_touch_line(
                        ResetParticleTouchLineRequest.TOUCH_LINE_RIGHT_TOP)
                else:
                    self.bb.reset_particle_touch_line(
                        ResetParticleTouchLineRequest.TOUCH_LINE_RIGHT_BOTTOM)
        else:
            self.bb.reset_particle_point(self.bb.start_pos.to_vector3())
        print("set ini point")
        return Status.RUNNING


class InitialGoaliePos(Skill):
    def execute(self):
        if self.bb.param.attack_right:
            self.bb.reset_particle_point(Vector3(-450, 0, 0))
        else:
            self.bb.reset_particle_point(Vector3(-450, 0, 0))

        # self.look_at(15, 0)
        # self.crouch()

        return Status.RUNNING


class SetOrientation(Skill):
    def execute(self):
        self.bb.set_orientation(self.bb.start_pos.z)
        return Status.RUNNING
