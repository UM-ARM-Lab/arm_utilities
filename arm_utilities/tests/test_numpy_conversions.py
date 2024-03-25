from arm_utilities.numpy_conversions import quat_to_rot_mat, rot_mat_to_quat, transform_to_mat, mat_to_transform

import numpy as np

from geometry_msgs.msg import Transform, Quaternion


def test_quat_to_rot_mat():
    q = Quaternion()
    q.x = 0.5
    q.y = 0.5
    q.z = 0.5
    q.w = 0.5
    expected = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
    assert np.allclose(quat_to_rot_mat(q), expected)


def test_rot_mat_to_quat():
    mat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    expected = np.array([0., 0., 0.70710678, 0.70710678])
    q = rot_mat_to_quat(mat)
    assert np.allclose([q.x, q.y, q.z, q.w], expected)


def test_transform_to_mat():
    t = Transform()
    t.translation.x = 1.
    t.translation.y = 2.
    t.translation.z = 3.
    t.rotation.x = 0.
    t.rotation.y = 1.
    t.rotation.z = 0.
    t.rotation.w = 0.
    expected = np.array([[-1, 0, 0, 1], [0, 1, 0, 2], [0, 0, -1, 3], [0, 0, 0, 1]], dtype=float)
    assert np.allclose(transform_to_mat(t), expected)


def test_mat_to_transform():
    mat = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]], dtype=float)
    expected = np.array([1., 2., 3., 0., 0., 0.70710678, 0.70710678], dtype=float)
    t = mat_to_transform(mat)
    assert np.allclose(
        [t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
        expected)


def test_to_and_from_rot_mat():
    rot_mat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    assert np.allclose(quat_to_rot_mat(rot_mat_to_quat(rot_mat)), rot_mat)


def test_to_and_from_mat():
    mat = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]], dtype=float)
    assert np.allclose(transform_to_mat(mat_to_transform(mat)), mat)
