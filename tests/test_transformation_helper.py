import numpy as np

from arm_utilities.transformation_helper import vector3_to_spherical, spherical_to_vector3, np_tf_inv


def test_np_tf_inv_identity():
    mat = np.eye(4)
    inv = np_tf_inv(mat)
    np.testing.assert_allclose(mat, inv, rtol=1e-6)


def test_np_tf_inv_random():
    rng = np.random.RandomState(0)
    for i in range(100):
        # Generate a random homogeneous transform matrix
        mat = np.eye(4)
        mat[0:3, 3] = rng.rand(3)
        # A random rotation matrix is generated using the QR decomposition
        a = rng.rand(3, 3)
        q, r = np.linalg.qr(a[0:3, 0:3])
        mat[0:3, 0:3] = q

        inv = np_tf_inv(mat)
        np.testing.assert_allclose(np.eye(4), mat@inv, atol=1e-6)
        np.testing.assert_allclose(np.eye(4), inv@mat, atol=1e-6)


def test_vector3_to_spherical_zero():
    xyz = [0, 0, 0]
    r_phi_theta = vector3_to_spherical(xyz)
    xyz_out = spherical_to_vector3(r_phi_theta)
    np.testing.assert_allclose(xyz, xyz_out, rtol=1)


def test_vector3_to_spherical_case1():
    xyz = [1, 0, 0]
    r_phi_theta = vector3_to_spherical(xyz)
    xyz_out = spherical_to_vector3(r_phi_theta)
    np.testing.assert_allclose(xyz, xyz_out, rtol=1)


def test_vector3_to_spherical_case2():
    xyz = [0, 1, 0]
    r_phi_theta = vector3_to_spherical(xyz)
    xyz_out = spherical_to_vector3(r_phi_theta)
    np.testing.assert_allclose(xyz, xyz_out, rtol=1)


def test_vector3_to_spherical_case3():
    xyz = [0, 0, 1]
    r_phi_theta = vector3_to_spherical(xyz)
    xyz_out = spherical_to_vector3(r_phi_theta)
    np.testing.assert_allclose(xyz, xyz_out, rtol=1)


def test_vector3_to_spherical_case4():
    xyz = [0, 1, 1]
    r_phi_theta = vector3_to_spherical(xyz)
    xyz_out = spherical_to_vector3(r_phi_theta)
    np.testing.assert_allclose(xyz, xyz_out, rtol=1)


def test_vector3_to_spherical_random():
    rng = np.random.RandomState(0)
    for i in range(100):
        xyz = rng.uniform(-2, 1, size=[3])
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)
