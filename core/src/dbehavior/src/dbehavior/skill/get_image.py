from dbehavior.core import Skill
from dbehavior.btree.core import Status
from dbehavior.util import Parameter

# init rotating path
CUR_POSE = (0, 0)
PARAM = Parameter()
MAX_YAW = PARAM.max_yaw
MAX_PITCH = PARAM.max_pitch
MIN_PITCH = PARAM.min_pitch
DIFF_YAW = [i for i in range(-MAX_YAW, MAX_YAW, 10)]
DIFF_PITCH = [i for i in range(MIN_PITCH, MAX_PITCH, 10)]
PATH = []
for i in range(len(DIFF_PITCH)):
    tmp = [DIFF_PITCH[i] for j in range(len(DIFF_YAW))]
    PATH += zip(DIFF_YAW, tmp)
    DIFF_YAW.reverse()
ITERATION = iter(PATH)


def reset_iteration():
    global CUR_POSE
    global ITERATION
    CUR_POSE = (0, 0)
    ITERATION = iter(PATH)


def get_cur_pose():
    global CUR_POSE
    return CUR_POSE


class GetImageRotate(Skill):
    """
    Rotate head when getting images.
    """

    def execute(self):
        global CUR_POSE
        self.look_at(pitch=CUR_POSE[1], yaw=CUR_POSE[0])
        return Status.RUNNING


class GetImageCapture(Skill):
    """
    Capture frame when getting images.
    """

    def execute(self):
        print('Cheese! ({}, {})'.format(CUR_POSE[1], CUR_POSE[0]))
        self.bb.behavior_info.save_image = True
        return Status.RUNNING

    def on_end(self):
        global CUR_POSE
        global ITERATION
        try:
            CUR_POSE = next(ITERATION)
        except StopIteration:
            reset_iteration()
