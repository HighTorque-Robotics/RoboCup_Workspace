from dbehavior.core import Skill
from dbehavior.util import VecPos
from dbehavior.btree.core import Status

def AlignToBall(Skill):

    def __init__(self, bb):
        super(AlignToBall, self).__init__(bb)
        # target [cm]
        # self.target = VecPos()
    
    def get_align_point(self, ref):
        """
        Get align point

        :param ref: reference point
        :type ref: VecPos
        :return: align point
        :rtype: VecPos,
        """
        robot_pos = self.bb.vision_info.robot_pos
        ball_global = self.bb.ball_global
        attack_right = self.bb.param.attack_right

        # if ball is in the goal, go to the home position
        if (attack_right and ball_global.x < -self.bb.param.field_length / 2.) or \
                (not attack_right and ball_global.x > self.bb.param.field_length / 2.):
            return self.secure_point(ref)

        # compute  y value on x coordinate using line: ref / ball
        d = self.intersect(ref, ball_global, robot_pos.x)
        if attack_right:
            d.z = 0
        else:
            d.z = 180

        # # correction
        # v = min(1., fabs(d.y) / self.bb.param.goal_width / 2.)
        # correct_factor = (-(2 * v - 1) * (2 * v - 1) + 1) * 15
        # if d.y > 0:
        #     d.y += correct_factor
        # else:
        #     d.y -= correct_factor

        return self.secure_point(d)
    
    def excute(self):

        # TODO: Goto align line(area), then forward/backward to best point
        # p = self.get_align_point(ball, )

        # if self._got_dest_simple(p):
        #     self.crouch()
        #     return Status.SUCCEEDED
        # else:
        #     self.goto_global(p)
        #     return Status.RUNNING
        return Status.RUNNING

