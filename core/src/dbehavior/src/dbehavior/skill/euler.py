from dbehavior.btree.core import Status
from dbehavior.core import Skill
from dbehavior.util.mathutil import degrees


class Euler(Skill):

    def execute(self):
        rpy = self.bb.motion_info.imuRPY

        print('---------')
        print('[imu] roll: {:2.2f} pitch: {:2.2f} yaw: {:2.2f}'.format(degrees(rpy.x),
                                                                       degrees(rpy.y),
                                                                       degrees(rpy.z)))

        return Status.SUCCEEDED
