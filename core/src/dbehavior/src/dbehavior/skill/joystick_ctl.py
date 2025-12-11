from dbehavior.core import Skill
from dbehavior.btree.core import Status
import rospy


class JoystickController(Skill):
    """
    Action controlled by joystick (Xbox 360 Wired Controller).

    **Table of index number of /joy.buttons**

    +-------+------------------------------------------------+
    | Index | Button name on the actual controller [buttons] |
    +=======+================================================+
    |   0   | A                                              |
    +-------+------------------------------------------------+
    |   1   | B                                              |
    +-------+------------------------------------------------+
    |   2   | X                                              |
    +-------+------------------------------------------------+
    |   3   | Y                                              |
    +-------+------------------------------------------------+
    |   4   | LB=left kick                                   |
    +-------+------------------------------------------------+
    |   5   | RB=left kick                                   |
    +-------+------------------------------------------------+
    |   6   | back                                           |
    +-------+------------------------------------------------+
    |   7   | start=crouch                                   |
    +-------+------------------------------------------------+
    |   8   | power                                          |
    +-------+------------------------------------------------+
    |   9   | Button stick left                              |
    +-------+------------------------------------------------+
    |   10  | Button stick right                             |
    +-------+------------------------------------------------+

    **Table of index number of /joy.axis**

    +-------+-------------------------------------------+
    | Index | Axis name on the actual controller [axes] |
    +=======+===========================================+
    |   0   | Left/Right Axis stick left                |
    +-------+-------------------------------------------+
    |   1   | Up/Down Axis stick left                   |
    +-------+-------------------------------------------+
    |   2   | LT                                        |
    +-------+-------------------------------------------+
    |   3   | Left/Right Axis stick right               |
    +-------+-------------------------------------------+
    |   4   | Up/Down Axis stick right                  |
    +-------+-------------------------------------------+
    |   5   | RT                                        |
    +-------+-------------------------------------------+
    |   6   | cross key left/right                      |
    +-------+-------------------------------------------+
    |   7   | cross key up/down                         |
    +-------+-------------------------------------------+
    |   8   | power                                     |
    +-------+-------------------------------------------+
    |   9   | Button stick left                         |
    +-------+-------------------------------------------+
    |   10  | Button stick right                        |
    +-------+-------------------------------------------+

    **Sample**

    header:
      seq: 3401
      stamp:
        secs: 1494076054
        nsecs: 484258726
      frame_id: ''
    axes: [0.00911218486726284, 0.026555921882390976, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    """

    def __init__(self, bb, head_ctl=True):
        super(JoystickController, self).__init__(bb)
        self.usage = \
            """
Reading from the joystick and Publishing to Motion!
---------------------------
Left Rocker: Walk 
Right Rocker: Adjust camera pose
LT/RT: Turn left/right
LB/RB: Kick left/right
BACK : crouch
START: step

CTRL-C to quit
            """
        print(self.usage)
        self.OFFSET_T = -7.5
        self.MAX_PITCH = 70
        self.MAX_YAW = 120
        self.HEAD_CTL = head_ctl

    def execute(self):
        signal = self.bb.joy_info
        if signal is None:
            return Status.FAILED

        # left rocker
        y, x = signal.axes[0:2]
        x = x * 5
        y = y * 5

        # right rocker
        yaw, pitch = signal.axes[3:5]
        yaw = yaw * self.MAX_YAW
        pitch = max(0, pitch * self.MAX_PITCH)

        # LT/RT
        LT, RT = signal.axes[2], signal.axes[5]
        LT = (1.0 - LT) / 2.0
        RT = (1.0 - RT) / 2.0
        t = (LT - RT) * 15.0

        # do action
        if self.HEAD_CTL:
            self.look_at(pitch, yaw)
        if signal.buttons[4]:
            rospy.loginfo("Pushing LB")
            self.kick_left()
        elif signal.buttons[5]:
            rospy.loginfo("Pushing RB")
            self.kick_right()
        elif signal.buttons[6]:
            rospy.loginfo("crouch")
            self.crouch()
        elif signal.buttons[7]:
            rospy.loginfo("step")
            self.step()
        elif abs(x) > 1 or abs(y) > 1:
            self.walk(x, y, t)
            rospy.loginfo("(x: %lf y: %lf t: %lf | pitch: %lf yaw: %lf" %
                          (x, y, t, pitch, yaw))
        else:
            self.crouch()

        return Status.RUNNING


# class JoystickSwitch(Skill):
#
#     def __init__(self, bb):
#         super(JoystickSwitch, self).__init__(bb)
#         self.usage = \
#             """
# Play and pause by control of joystick!
# ---------------------------
# START: Do skill
# BACK: Just crouch
#
# CTRL-C to quit
#             """
#         print(self.usage)
#
#     def execute(self):
#         signal = self.bb.joy_info
#         if signal is not None:
#             if self.joy_info.buttons[7]:  # 'start' button
#                 self.bb.joystick_started = True
#             elif self.joy_info.buttons[6]:  # 'back' button
#                 self.bb.joystick_started = False
#             rospy.loginfo("Started: {}".format(self.bb.joystick_started))
#
#         return Status.RUNNING
