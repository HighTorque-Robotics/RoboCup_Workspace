from dmsgs.msg import BodyCommand
from dmsgs.msg import HeadCommand


class ActionGenerator(object):
    """
    Generator for action command.
    """

    @staticmethod
    def head(pitch=0, yaw=0, pitch_speed=2, yaw_speed=2):
        """
        Returns Head command.

        :param pitch: desired pitch value (in degree) of camera pose
        :type pitch: float
        :param yaw: desired yaw value (in degree) of camera pose
        :type yaw: float
        :param pitch_speed: rotation speed for adjusting pitch value
        :type pitch_speed: float
        :param yaw_speed: rotation speed for adjusting yaw value
        :type yaw_speed: float
        :return: head command corresponding to given parameters
        :rtype: HeadCommand
        """
        return HeadCommand(pitch, yaw, pitch_speed, yaw_speed)

    @staticmethod
    def crouch():
        """
        Returns Body Command with gait type for crouching action

        :return: body command for crouching action
        :rtype: BodyCommand
        """
        return BodyCommand(0, 0, 0, BodyCommand.CROUCH)
        

    @staticmethod
    def walk_pos(x=0, y=0, z=0):
        """
        Returns Body Command with gait type for walking to a give nposition action

        :param x: x(cm) of target a given position
        :type x: float
        :param y: y(cm) of target a given position
        :type y: float
        :param z: z(angle in degree) of target a given position
        :type z: float
        :return: body command for walking to a given position action
        :rtype: BodyCommand
        """
        return BodyCommand(x, y, z, BodyCommand.WALK_POS)

    @staticmethod
    def kick_ball(x=0, y=0, z=0):
        """
        Returns Body Command with gait type for walking to ball action

        :param x: x(cm) of ball position
        :type x: float
        :param y: y(cm) of ball position
        :type y: float
        :param z: z(angle in degree) of target kick direction
        :type z: float
        :return: body command for walking to ball action
        :rtype: BodyCommand
        """
        return BodyCommand(x, y, z, BodyCommand.KICK_BALL)

    @staticmethod
    def dribble(x=0, y=0, z=0):
        """
        Returns Body Command with gait type for walking to ball action

        :param x: x(cm) of ball position
        :type x: float
        :param y: y(cm) of ball position
        :type y: float
        :param z: z(angle in degree) of target dribble direction
        :type z: float
        :return: body command for walking to ball action
        :rtype: BodyCommand
        """
        return BodyCommand(x, y, z, BodyCommand.DRIBBLE)


    @staticmethod
    def turn(t=0):
        """
        Returns Body Command with gait type for turning to target direction

        :param t: t(angle in degree) of target direction
        :type t: float
        :return: body command for walking to ball action
        :rtype: BodyCommand
        """
        # return BodyCommand(0, 0, t, BodyCommand.WALK_POS)
        return BodyCommand(0, 0, t, BodyCommand.TURN)

    @staticmethod
    def backward():
        """
        Returns Body Command with gait type for go backward

        :return: body command for walking to ball action
        :rtype: BodyCommand
        """
        return BodyCommand(0, 0, 0, BodyCommand.WALK_POS)

    @staticmethod
    def goalie_mid():
        """
    Returns Body Command with gait type for goalie's middle defence action

        :return: body command for goalie's middle defence action
        :rtype: BodyCommand
        """
        return BodyCommand(0, 0, 0, BodyCommand.WALK_POS)

    @staticmethod
    def goalie_left():
        """
        Returns Body Command with gait type for goalie's left defence action

        :return: body command for goalie's left defence action
        :rtype: BodyCommand
        """
        return BodyCommand(0, 0, 0, BodyCommand.WALK_POS)

    @staticmethod
    def goalie_right():
        """
        Returns Body Command with gait type for goalie's right defence action

        :return: body command for goalie's right defence action
        :rtype: BodyCommand
        """
        return BodyCommand(0, 0, 0, BodyCommand.WALK_POS)
