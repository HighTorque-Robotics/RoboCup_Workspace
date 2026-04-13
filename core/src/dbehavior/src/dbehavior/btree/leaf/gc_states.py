from dbehavior.btree.leaf import ConditionLeaf
from dmsgs.msg import GCInfo
from dbehavior.util import VecPos


class GCConnected(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.connected


class GCLost(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_lost


class GCConnectionInited(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_connection_inited


class GCInitial(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_INITIAL


class GCReady(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_READY


class GCSet(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_SET


class GCPlaying(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_PLAYING and \
               not self.get_bb().gc_info.stopped


class GCFinished(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_FINISHED


class GCReEntry(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state == GCInfo.STATE_PLAYING and \
               not self.get_bb().gc_info.stopped and \
               self.get_bb().timer_re_entry.elapsed() <= 10


class GCInitialEntry(ConditionLeaf):

    def condition(self):
        return (self.get_bb().gc_info.penalised or
                self.get_bb().gc_info.state in [
                    GCInfo.STATE_INITIAL, GCInfo.STATE_READY, GCInfo.STATE_SET
                ])

# --- [修改] 统一判断是否处于任何定位球状态 ---
class GCFreeKick(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.setPlay in [
            GCInfo.SET_PLAY_DIRECT_FREE_KICK,
            GCInfo.SET_PLAY_INDIRECT_FREE_KICK,
            GCInfo.SET_PLAY_PENALTY_KICK,
            GCInfo.SET_PLAY_CORNER_KICK,
            GCInfo.SET_PLAY_GOAL_KICK,
            GCInfo.SET_PLAY_THROW_IN
        ] or self.get_bb().enemy_free_kick


# --- [修改] 点球大战现在由 gamePhase 决定 ---
class GCPenaltyShoot(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.gamePhase == GCInfo.GAME_PHASE_PENALTY_SHOOT_OUT


# --- [修改] 暂停状态现在由 gamePhase 决定 ---
class GCTimeout(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.gamePhase == GCInfo.GAME_PHASE_TIMEOUT


# --- [修改] 常规比赛必须满足大阶段为 NORMAL 且没有任何定位球 (NONE) ---
class GCNormal(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.gamePhase == GCInfo.GAME_PHASE_NORMAL and \
               gc_info.setPlay == GCInfo.SET_PLAY_NONE


class GCOurFreeKick(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.ourIndirectFreeKick or \
               gc_info.ourDirectFreeKick or \
               gc_info.ourCornerKick or \
               gc_info.ourGoalKick or \
               gc_info.ourThrowIn or \
               gc_info.ourPenaltyKick


class GCEnemyFreeKick(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.enemyIndirectFreeKick or \
               gc_info.enemyDirectFreeKick or \
               gc_info.enemyPenaltyKick or \
               gc_info.enemyCornerKick or \
               gc_info.enemyGoalKick or \
               gc_info.enemyThrowIn


class GCOurCornerKick(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.ourCornerKick


class GCEnemyCornerKick(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.enemyCornerKick


class GCOurGoalKick(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.ourGoalKick


class GCEnemyGoalKick(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.enemyGoalKick


# --- [修改] 修复了界外球错误包含角球和球门球的 Bug ---
class GCOurThrowIn(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.ourThrowIn


class GCEnemyThrowIn(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.enemyThrowIn


class GCState2Freeze(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state2Freeze


class GCState2Ready(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.state2Ready


class GCPenalised(ConditionLeaf):

    def condition(self):
        return self.get_bb().gc_info.penalised


# --- [修改] Normal 系列节点增加对 gamePhase 和 setPlay 的双重检查 ---
class GCNormalInitial(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_INITIAL and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_NORMAL and \
               gc_info.setPlay == GCInfo.SET_PLAY_NONE


class GCNormalReady(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_READY and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_NORMAL and \
               gc_info.setPlay == GCInfo.SET_PLAY_NONE


class GCNormalSet(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_SET and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_NORMAL and \
               gc_info.setPlay == GCInfo.SET_PLAY_NONE


class GCNormalPlaying(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_PLAYING and \
               not gc_info.stopped and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_NORMAL and \
               gc_info.setPlay == GCInfo.SET_PLAY_NONE


# --- [修改] PenaltyShoot 系列节点改用 gamePhase ---
class GCPenaltyShootInitial(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_INITIAL and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_PENALTY_SHOOT_OUT


class GCPenaltyShootSet(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_SET and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_PENALTY_SHOOT_OUT


class GCPenaltyShootPlaying(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.state == GCInfo.STATE_PLAYING and \
               not gc_info.stopped and \
               gc_info.gamePhase == GCInfo.GAME_PHASE_PENALTY_SHOOT_OUT


class GCKickOffSupporterDelay(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        # 增加拦截：必须是常规比赛阶段且没有任何定位球
        if gc_info.gamePhase != GCInfo.GAME_PHASE_NORMAL or gc_info.setPlay != GCInfo.SET_PLAY_NONE:
            return False
            
        return gc_info.kickoff and \
               self.get_bb().param.pos_role == 'supporter' and \
               gc_info.state == GCInfo.STATE_PLAYING and \
               not gc_info.stopped and \
               gc_info.secondaryTime != 0


class GCNonKickOffPlaying(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        # 增加拦截：必须是常规比赛阶段且没有任何定位球
        if gc_info.gamePhase != GCInfo.GAME_PHASE_NORMAL or gc_info.setPlay != GCInfo.SET_PLAY_NONE:
            return False
            
        striker_delay = not gc_info.kickoff and \
                        self.get_bb().param.pos_role == 'striker' and \
                        gc_info.state == GCInfo.STATE_PLAYING and \
                        not gc_info.stopped and \
                        gc_info.secondaryTime > 3
        supporter_delay = not gc_info.kickoff and \
                        self.get_bb().param.pos_role == 'supporter' and \
                        gc_info.state == GCInfo.STATE_PLAYING and \
                        not gc_info.stopped and \
                        gc_info.secondaryTime > 8
        return striker_delay or supporter_delay


class GCNonKickOffPending(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        # 增加拦截：必须是常规比赛阶段且没有任何定位球
        if gc_info.gamePhase != GCInfo.GAME_PHASE_NORMAL or gc_info.setPlay != GCInfo.SET_PLAY_NONE:
            return False
            
        robot_pos = VecPos.from_vector3(self.get_bb().vision_info.robot_pos)
        see_circle = self.get_bb().vision_info.see_circle
        circle_field = VecPos.from_vector3(
            self.get_bb().vision_info.circle_field)

        non_kick_off = not gc_info.kickoff and \
                       gc_info.state == GCInfo.STATE_PLAYING and \
                       not gc_info.stopped and \
                       self.get_bb().timer_gc_playing.elapsed() <= 10
        pos_near_circle = \
            robot_pos.length() <= self.get_bb().param.center_circle_diameter / 2.0 + 20
        circle_field_near = \
            see_circle and circle_field.length() <= self.get_bb().param.center_circle_diameter / 2.0 + 10

        return non_kick_off and (pos_near_circle or circle_field_near)


# --- [修改] 界外球逻辑现在由 setPlay 判定 ---
class GCThrowInWait(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.setPlay == GCInfo.SET_PLAY_THROW_IN and \
               gc_info.secondaryTime < 1


class GCThrowInPlacing(ConditionLeaf):

    def condition(self):
        gc_info = self.get_bb().gc_info
        return gc_info.setPlay == GCInfo.SET_PLAY_THROW_IN and \
               gc_info.secondaryTime >= 1


class GCBallNotInPlay(ConditionLeaf):

    def condition(self):
        # NOTE(daiz): Less wait time(8s) for log of start walking.
        return self.get_bb().timer_state2_normal.elapsed() < 8