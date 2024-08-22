from typing import Union

import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from geometry_msgs.msg import Transform, Pose


def np_tf_inv(mat):
    """
    Specialized case of matrix inversion for a homogeneous transform

    Args:
        mat: 4x4 numpy array
    """
    inv = np.zeros((4, 4))
    inv[0:3, 0:3] = mat[0:3, 0:3].T
    inv[0:3, 3] = -np.dot(mat[0:3, 0:3].T, mat[0:3, 3])
    inv[3, :] = [0, 0, 0, 1]
    return inv


def spherical_to_vector3(r_phi_theta):
    """
    @param r_phi_theta: list-like object [r, phi, theta]
    @return: list of x, y, z
    """
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
    r, phi, theta = r_phi_theta
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return [x, y, z]


def vector3_to_spherical(xyz):
    """
    @param xyz: list-like object [x, y, z]
    @return: list of r, phi, theta
    """
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
    x, y, z = xyz
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    # phi is the angle about the z axis, where 0 is +x and pi/2 is +y
    phi = np.arctan2(y, x)
    # theta is inclination from z direction
    if r > 1e-6:
        theta = np.arccos(z / r)
    else:
        theta = 0
    return [r, phi, theta]


def build_mat(translation, quaternion):
    """
    @param translation: list-like object [x, y, z]
    @param quaternion: list-like object [x, y, z, w]
    @return: 4x4 numpy array
    """
    mat = np.eye(4)
    mat[0:3, 0:3] = quat2mat(np.roll(quaternion, 1))
    mat[0:3, 3] = translation
    return mat


def build_mat_from_transform(transform: Transform):
    """
    @param transform: geometry_msgs.msg.Transform
    @return: 4x4 numpy array
    """
    translation = [transform.translation.x, transform.translation.y, transform.translation.z]
    quaternion = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    return build_mat(translation, quaternion)


def get_vec7_from_transform(transform: Union[Transform, Pose]):
    """
    @param transform: geometry_msgs.msg.Transform
    @return: list of [x, y, z, qx, qy, qz, qw]
    """
    if isinstance(transform, Transform):
        return [transform.translation.x, transform.translation.y, transform.translation.z,
                transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    elif isinstance(transform, Pose):
        return [transform.position.x, transform.position.y, transform.position.z,
                transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w]

def extract_from_matrix(mat):
    """
    @param mat: 4x4 numpy array
    @return: list of translation [x, y, z] and quaternion [x, y, z, w]
    """
    translation = mat[0:3, 3]
    quaternion = mat2quat(mat[0:3, 0:3])
    quaternion = np.roll(quaternion, -1).tolist()
    return translation, quaternion
