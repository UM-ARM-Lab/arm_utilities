import numpy as np

from geometry_msgs.msg import Transform, Quaternion


def quat_to_rot_mat(q: Quaternion):
    """ Convert a geometry_msgs/Quaternion to a 3x3 numpy array """
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                     [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
                     [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]])


def rot_mat_to_quat(mat: np.ndarray):
    """ Convert a 3x3 numpy array to a geometry_msgs/Quaternion """
    w = np.sqrt(1 + mat[0, 0] + mat[1, 1] + mat[2, 2]) / 2
    x = (mat[2, 1] - mat[1, 2]) / (4 * w)
    y = (mat[0, 2] - mat[2, 0]) / (4 * w)
    z = (mat[1, 0] - mat[0, 1]) / (4 * w)
    return Quaternion(x=x, y=y, z=z, w=w)


def transform_to_mat(transform: Transform):
    """ Convert a geometry_msgs/Transform to a 4x4 numpy array """
    mat = np.eye(4)
    mat[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    mat[:3, :3] = quat_to_rot_mat(transform.rotation)
    return mat


def mat_to_transform(mat: np.ndarray):
    """ Convert a 4x4 numpy array to a geometry_msgs/Transform """
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = mat[:3, 3]
    t.rotation = rot_mat_to_quat(mat[:3, :3])
    return t
