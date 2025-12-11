from dbehavior.util import Timer, VecPos
from dbehavior.core import Skill
from dbehavior.btree.core import Status
from math import sqrt

keepPointLeft = VecPos(-600, 0, 0)
keepPointRight = VecPos(600, 0, 180)
keepLineLeft = VecPos(-440, 0, 0)
keepLineRight = VecPos(440, 0, 180)
scrambleDist = 30  # the ball distance to decide which side to scramble
dangerDist = 70  # the ball distance to take action by ball velocity
maxKeepUp = 80
minKeepDown = -80
dangerVelocity = 100


class GoalieBodyLineUp(Skill):
    def __init__(self, bb):
        super(GoalieBodyLineUp, self).__init__(bb)
        self.left_kick = False
        self.keepPoint = VecPos(0, 0, 0)
        self.attackPointY = 0
        self.dx = 0
        self.dy = 0

    def execute(self):
        ball_velocity = VecPos.from_vector3(self.bb.vision_info.ball_velocity)
        if self.bb.vision_info.see_ball:
            if abs(self.bb.vision_info.ball_field.x) < scrambleDist:
                if self.bb.vision_info.ball_field.y > 5:
                    print 'scramble left --- scrambleDist'
                    return Status.FAILED  # fail
                elif self.bb.vision_info.ball_field.y < -5:
                    print 'scramble right --- scrambleDist'
                    return Status.FAILED  # fail
                else:
                    print 'keep mid --- scrambleDist'
                    self.crouch()
                    return Status.RUNNING
            elif abs(self.bb.vision_info.ball_field.x
                     ) < dangerDist and ball_velocity.length() != 0:
                print 'ball velocity : ', ball_velocity
                self.calcAttackPointY()
                if ball_velocity.length > dangerVelocity and self.attackPointY > 5:
                    print 'scramble left --- dangerDist'
                    return Status.FAILED
                elif ball_velocity.length > dangerVelocity and self.attackPointY < -5:
                    print 'scramble right --- dangerDist'
                    return Status.FAILED
                else:
                    print 'keep mid --- dangerDist'
                    self.crouch()
                    return Status.RUNNING
            else:
                if self.calPos():
                    if self.distance() > 10:

                        self.walk(self.dx, self.dy, 0)
                    else:
                        self.crouch()
                    return Status.RUNNING
                else:
                    return Status.RUNNING
        return Status.RUNNING

    def calcAttackPointY(self):
        self.attackPointY = ball_velocity.y / ball_velocity.x * self.bb.vision_info.ball_field.x + self.bb.vision_info.ball_field.y

    def calPos(self):
        if abs(self.bb.vision_info.ball_global.x - keepPointLeft.x) < 10:
            return False
        if not self.left_kick:
            self.keepPoint.y = (
                self.bb.vision_info.ball_global.y - keepLineLeft.y) * (
                    keepLineLeft.x - keepPointLeft.x) / (
                        self.bb.vision_info.ball_global.x - keepPointLeft.x
                    ) + keepLineLeft.y
            self.keepPoint.x = keepLineLeft.x
            self.keepPoint.z = 0
        else:
            self.keepPoint.y = (
                self.bb.vision_info.ball_global.y - keepLineRight.y) * (
                    keepLineRight.x - keepPointRight.x) / (
                        self.bb.vision_info.ball_global.x - keepPointRight.x
                    ) + keepLineRight.y
            self.keepPoint.x = keepLineRight.x
            self.keepPoint.z = -180
        if self.keepPoint.y > maxKeepUp:
            self.keepPoint.y = maxKeepUp
        elif self.keepPoint.y < minKeepDown:
            self.keepPoint.y = minKeepDown
        return True

    def distance(self):
        x = self.bb.vision_info.robot_pos.x
        y = self.bb.vision_info.robot_pos.y
        self.dx = self.keepPoint.x - x
        self.dy = self.keepPoint.y - y
        return sqrt(self.dx * self.dx + self.dy * self.dy)


class GoalieDefend(Skill):
    def __init__(self, bb):
        super(GoalieDefend, self).__init__(bb)
        self.left_kick = False
        self.dy = 0

    def execute(self):
        ball_velocity = VecPos.from_vector3(self.bb.vision_info.ball_velocity)
        # TODO(daiz) try the effect of using velocity
        # if ball_velocity.x != 0:
        # self.dy = ball_velocity.y / ball_velocity.x * self.bb.ball_field.x + self.bb.ball_field.y
        # else:
        # if not self.left_kick:
        #     self.dy = (self.bb.ball_global.y - keepLineLeft.y) * (
        #         keepLineLeft.x - keepPointLeft.x) / (
        #             self.bb.ball_global.x - keepPointLeft.x) + keepLineLeft.y
        # else:
        #     self.dy = (self.bb.ball_global.y - keepLineRight.y) * (
        #         keepLineRight.x - keepPointRight.x) / (
        # self.bb.ball_global.x - keepPointRight.x) + keepLineRight.y
        if self.bb.vision_info.ball_field.y > 0:
            dy = 1
        else:
            dy = -1
        if dy > 0:
            print 'goalie_left()'
            self.goalie_left()
        else:
            print 'goalie_right()'
            self.goalie_right()
        return Status.RUNNING
