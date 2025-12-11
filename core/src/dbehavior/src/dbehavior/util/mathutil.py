from math import pi, sqrt, atan2, degrees, radians, sin, cos, floor

PI = pi
PI_2 = pi / 2.0
PI_4 = pi / 4.0
PI_1_3 = pi / 3.0

Nan = float('nan')
Inf = float('inf')


def mirror(vector):
    vector.x *= -1
    vector.y *= -1
    vector.z += 180
    vector.z = angle_normalization(vector.z)
    return vector

def cosd(degree):
    """
    Calculate cosine value of angle in degree.

    :param degree: given angel in degree
    :type degree: float
    :return: cosine value of angle in degree.
    :rtype: float
    """
    return cos(radians(degree))


def sind(degree):
    """
    Calculate sine value of angle in degree.

    :param degree: given angel in degree
    :type degree: float
    :return: sine value of angle in degree.
    :rtype: float
    """
    return sin(radians(degree))


def get_angle(vector):
    """
    Calculate angle between x axis and given vector.
    :param vector: given vector
    :type vector: VecPos
    :return: angle between x axis and given vector.
    :rtype: float
    """
    return degrees(atan2(vector.y, vector.x))


def get_dis(vec_1, vec_2):
    """
    Calculate distance of points represented by given vector.
    :param vec_1: first given vector
    :type vec_1: VecPos
    :param vec_2: second given vector
    :type vec_2: VecPos
    :return: distance of points represented by given vector.
    :rtype: float
    """
    dx = vec_1.x - vec_2.x
    dy = vec_1.y - vec_2.y
    return sqrt(dx * dx + dy * dy)


def get_magnitude(vec):
    """
    Calculate magnitude of given vector.
    :param vec: given vector
    :type vec: VecPos
    :return: magnitude of given vector.
    :rtype: float
    """
    return sqrt(vec.x * vec.x + vec.y * vec.y)


def angle_between(vec_1, vec_2):
    """
    Calculate angle (in radian) between two vectors.
    
    :param vec_1: first given vector
    :type vec_1: VecPos
    :param vec_2: second given vector
    :type vec_2: VecPos
    :return: angel (in radian) between teo vectors.
    :rtype: float
    """
    return atan2(vec_2.y - vec_1.y, vec_2.x - vec_1.x)


def degree_between(vec_1, vec_2):
    """
    Calculate angle (in degree) between two vectors.

    :param vec_1: first given vector
    :type vec_1: VecPos
    :param vec_2: second given vector
    :type vec_2: VecPos
    :return: angel (in degree) between teo vectors.
    :rtype: float
    """
    return angle_normalization(degrees(angle_between(vec_1, vec_2)))


def angle_between2(vec_1, vec_2, vec_orig):
    """
    Calculate angle (in radian) between two vectors with one original vector.

    :param vec_1: first given vector
    :type vec_1: VecPos
    :param vec_2: second given vector
    :type vec_2: VecPos
    :param vec_orig: given vector as original point
    :type vec_orig: VecPos
    :return: angle (in radian) between two vectors with one original vector.
    :rtype: float
    """
    return angle_between(vec_1 - vec_orig, vec_2 - vec_orig)


def degree_between2(vec_1, vec_2, vec_orig):
    """
    Calculate angle (in degree) between two vectors with one original vector.

    :param vec_1: first given vector
    :type vec_1: VecPos
    :param vec_2: second given vector
    :type vec_2: VecPos
    :param vec_orig: given vector as original point
    :type vec_orig: VecPos
    :return: angle (in degree) between two vectors with one original vector.
    :rtype: float
    """
    return angle_normalization(degree_between2(vec_1, vec_2, vec_orig))


def angle_normalization(angle):
    """
    Normalize given angle in [-180, 180]

    :param angle: given angle (in degree)
    :type angle: float
    :return: normalized angle
    :rtype: float
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def radian_normalization(rad):
    """
    Normalize given angle in [-PI, PI]

    :param rad: given angle (in radian)
    :type rad: float
    :return: normalized angle
    :rtype: float
    """
    return rad - 2.0 * PI * floor((rad + PI) / (2.0 * PI))


def abs_angle_diff(angle):
    """
    Calculate absolute value of normalized angle.

    :param angle: given angle (in degree)
    :type angle: float
    :return: absolute value of normalized angle
    :rtype: float
    """
    return abs(angle_normalization(angle))


def sign(v):
    """
    Return result of given value processed by sign function.

    :param v: given value
    :type v: float
    :return: result value
    :rtype: float
    """
    if v > 0:
        return 1.0
    if v < 0:
        return -1.0
    return 0


def rad2deg(radian):
    """
    Convert angle from radian into degree.

    :param radian: angle in radian
    :type radian: float
    :return: angel in degree
    :rtype: float
    """
    return radian / PI * 180


def deg2rad(degree):
    """
    Convert angle from degree into radian.

    :param degree: angle in degree
    :type degree: float
    :return: angel in radian
    :rtype: float
    """
    return degree / 180 * PI