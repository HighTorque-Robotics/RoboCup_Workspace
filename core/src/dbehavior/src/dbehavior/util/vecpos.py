from math import atan2
from geometry_msgs.msg import Vector3

from dbehavior.util.mathutil import (cosd, sind, angle_normalization, get_dis, get_magnitude, rad2deg)


# TODO check whether or not to use in-place operation.


class VecPos(object):
    def __init__(self, x=0., y=0., angle=0.):
        """
        Initialize VecPos with given parameters.

        :param x: position x
        :type x: float
        :param y: position y
        :type y: float
        :param angle: direction
        :type angle: float
        """
        self.x = x
        self.y = y
        self.z = angle

    @classmethod
    def from_vector3(cls, vec):
        """
        Initialize VecPos from Vector3

        :param vec: given vector
        :type vec: Vector3
        :return: corresponding VecPos contructed from Vector3
        :rtype: VecPos
        """
        return cls(vec.x, vec.y, vec.z)

    @classmethod
    def from_list(cls, vec):
        if len(vec) == 2:
            return cls(vec[0], vec[1])
        elif len(vec) == 3:
            return cls(vec[0], vec[1], vec[2])
        else:
            return cls()

    def __str__(self):
        return "[{}, {}, {}]".format(self.x, self.y, self.z)

    def __add__(self, other):
        if isinstance(other, VecPos):
            # FIXME do we need the operation for z axis?
            # return VecPos(self.x + other.x, self.y + other.y, self.z + other.z)
            return VecPos(self.x + other.x, self.y + other.y)
        elif isinstance(other, int) or isinstance(other, float):
            # return VecPos(self.x + other, self.y + other, self.z + other)
            return VecPos(self.x + other, self.y + other, self.z + other)
        else:
            return NotImplemented
        
    def __iadd__(self, other):
        if isinstance(other, VecPos):
            self.x += other.x
            self.y += other.y
            # self.z += other.z
            return self
        elif isinstance(other, int) or isinstance(other, float):
            self.x += other
            self.y += other
            # self.z += other
            return self
        else:
            return NotImplemented

    def __sub__(self, other):
        if isinstance(other, VecPos):
            # return VecPos(self.x - other.x, self.y - other.y, self.z - other.z)
            return VecPos(self.x - other.x, self.y - other.y)
        elif isinstance(other, int) or isinstance(other, float):
            # return VecPos(self.x - other, self.y - other, self.z - other)
            return VecPos(self.x - other, self.y - other)
        else:
            return NotImplemented

    def __isub__(self, other):
        if isinstance(other, VecPos):
            self.x -= other.x
            self.y -= other.y
            # self.z -= other.z
            return self
        elif isinstance(other, int) or isinstance(other, float):
            self.x -= other
            self.y -= other
            # self.z -= other
            return self
        else:
            return NotImplemented

    def __eq__(self, other):
        if isinstance(other, VecPos):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return NotImplemented

    def __ne__(self, other):
        if isinstance(other, VecPos):
            return not self.__eq__(other)
        return NotImplemented

    def length(self):
        """
        Calculate length of vector.

        :return: length of vector
        :rtype: float
        """
        return get_magnitude(self)

    def distance(self, dest):
        """
        Calculate distance from current position to destination one.

        :param dest: destination position
        :type dest: VecPos
        :return: distance from current position to destination one
        :rtype: float
        """
        return get_dis(self, dest)

    def normalize(self, scale=1.):
        tmp = self.copy()
        len = tmp.length()
        tmp.x *= scale / len
        tmp.y *= scale / len
        return tmp

    def slope(self):
        """
        Calculate slope of vector.

        :return: slope of vector
        :rtype: float
        """
        return rad2deg(atan2(self.y, self.x))

    def copy(self):
        """
        Return copy of self.

        :return: copy
        :rtype: VecPos
        """
        return VecPos(self.x, self.y, self.z)

    def rotate(self, angle):
        """
        Rotate vector with given angle (in degree).

        :param angle: given angle (in degree)
        :type angle: float
        :return: rotated vector
        :rtype: VecPos
        """
        x = self.x
        y = self.y
        self.x = x * cosd(angle) - y * sind(angle)
        self.y = x * sind(angle) + y * cosd(angle)
        return self

    def dot(self, other):
        """
        Calculate dot product with given vector.

        :param other: given vector
        :type other: VecPos
        :return: product result
        :rtype: float
        """
        return self.x * other.x + self.y * other.y

    def to_vector3(self):
        """
        Convert VecPos to Vector3.

        :return: corresponding Vector3
        :rtype: Vector3
        """
        return Vector3(self.x, self.y, self.z)

    def mirror(self):
        """
        Mirror vector.

        :return: mirrored vector
        :rtype: VecPos
        """
        return VecPos(-self.x, -self.y, angle_normalization(self.z + 180.))

    def mirror_by_y_axis(self):
        """
        Mirror vector.

        :return: mirrored vector
        :rtype: VecPos
        """
        return VecPos(-self.x, self.y, angle_normalization(-self.z + 180.))

    @staticmethod
    def make(pos):
        """
        Construct a VecPos with given object.

        :param pos: given position vector
        :type pos: VecPos or Vector3
        :return: corresponding VecPos
        :rtype: VecPos
        """
        return VecPos(pos.x, pos.y)

    @staticmethod
    def calc_global_position(field_pos, robot_state):
        """
        From robot's global position and object's field position, get object's global position

        :param field_pos: position in robot coordinate
        :type field_pos: VecPos
        :param robot_state: robot position in field coordinate
        :type robot_state: VecPos
        :return: position in field coordinate
        :rtype: VecPos
        """
        tmp = VecPos(field_pos.x, field_pos.y)
        tmp.rotate(robot_state.z)
        return VecPos(tmp.x + robot_state.x, tmp.y + robot_state.y)

    @staticmethod
    def calc_field_position(global_pos, robot_state):
        """
        get the field position of an object with it's global position and robot's global position

        :param global_pos: position in field coordinate
        :type global_pos: VecPos
        :param robot_state: robot position in field coordinate
        :type robot_state: VecPos
        :return: position in robot coordinate
        :rtype: VecPos
        """
        tmp = VecPos(global_pos.x - robot_state.x, global_pos.y - robot_state.y)
        tmp.rotate(-robot_state.z)
        tmp.z = angle_normalization(global_pos.z - robot_state.z)
        return tmp
