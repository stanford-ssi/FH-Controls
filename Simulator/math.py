import numpy as np
import math 

_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> np.allclose(np.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> np.allclose(np.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj * si
        M[i, k] = sj * ci
        M[j, i] = sj * sk
        M[j, j] = -cj * ss + cc
        M[j, k] = -cj * cs - sc
        M[k, i] = -sj * ck
        M[k, j] = cj * sc + cs
        M[k, k] = cj * cc - ss
    else:
        M[i, i] = cj * ck
        M[i, j] = sj * sc - cs
        M[i, k] = sj * cc + ss
        M[j, i] = cj * sk
        M[j, j] = sj * ss + cc
        M[j, k] = sj * cs - sc
        M[k, i] = -sj
        M[k, j] = cj * si
        M[k, k] = cj * ci
    return M

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


"""
Coordinate Transformation functions, by Jerry
"""


def transform_ground_to_rocket(coordinates, rpy_angles):
    """
    Transforms ground frame coordinates to rocket frame coordinates.
    - Transformation matrix is (inverse Yaw) (inverse Pitch) (inverse Roll)
    - Recall yaw matrix is the matrix for rotation about z-axis by yaw angle;
    - Recall inverse of a rotation matrix is rotation about the same axis by negative of the rotation angle
    :param coordinates: (list[3]) coordinates in ground frame
    :param rpy_angles: (list[3]) z-axis, y-axis, x-axis rotations (i.e. yaw, pitch, roll), in this exact order
    :return: 3x1 np array (column vector of 3 elements) specifying coordinates in rocket frame
    """
    if len(coordinates) != 3 or len(rpy_angles) != 3:
        raise ValueError("List of angles not of right length (expected 3 values per list)!")

    # matrix product only works on column vectors. So reshape to a column vector
    ret_coordinates = np.array(coordinates).reshape(3, 1)
    # multiply by inverse of z-axis rotation Rot(yaw_angle, yaw_axis)
    ret_coordinates = np.matmul(rotation_matrix(-rpy_angles[0], 'z'), ret_coordinates)
    # multiply by inverse of y-axis rotation
    ret_coordinates = np.matmul(rotation_matrix(-rpy_angles[1], 'y'), ret_coordinates)
    # multiply by inverse of x-axis rotation
    ret_coordinates = np.matmul(rotation_matrix(-rpy_angles[2], 'x'), ret_coordinates)

    return ret_coordinates.reshape(3)


def transform_rocket_to_ground(coordinates, rpy_angles):
    """
    Transforms rocket frame coordinates to ground frame coordinates;
    - Inverse of transformation from rocket frame to ground frame
    - Hence, equal to (Yaw) (Pitch) (Roll), i.e. R_z R_y R_x
    :param coordinates: list of coordinates in ground frame
    :param rpy_angles: (list[3]) angles for rotation about z, y, x, IN THIS EXACT ORDER
    :return: 3x1 np array (column vector of 3 elements) specifying coordinates in rocket frame
    """
    if len(coordinates) != 3 or len(rpy_angles) != 3:
        raise ValueError("List of angles / axis not of right length (expected 3 values per list)!")

    # matrix product only works on column vectors. So reshape to a column vector
    ret_coordinates = np.array(coordinates).reshape(3, 1)
    # multiply by matrix for roll / rot. about X
    ret_coordinates = np.matmul(rotation_matrix(rpy_angles[2], 'x'), ret_coordinates)
    # multiply by matrix for pitch / rot. about Y
    ret_coordinates = np.matmul(rotation_matrix(rpy_angles[1], 'y'), ret_coordinates)
    # multiply by matrix for yaw / rot. about Z
    ret_coordinates = np.matmul(rotation_matrix(rpy_angles[0], 'z'), ret_coordinates)

    return ret_coordinates.reshape(3)


def rotation_matrix(angle, axis="z"):
    """
    Helper Function Rotation Matrix Function. Returns Rotation Matrix R(axes, angle)
    :param angle: rotation angle
    :param axis: axis to rotate about
    :return: 3d numpy array that specifies the rotation
    """
    if axis == "z":
        matrix = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])
        return matrix
    elif axis == "y":
        matrix = np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
        return matrix
    elif axis == "x":
        matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
        return matrix
    else:
        raise ValueError(f"'{axis}' is not a valid 3D rotation axis!")

