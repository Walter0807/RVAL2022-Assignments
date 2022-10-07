import numpy as np

def normalize(q):
    """
    Normalize quaternion q to unit length.
    Expects an array of shape (4, )
    Returns an array of shape (4, ).
    """
    raise NotImplementedError

def multiply(q1, q2):
    """
        Calculate the quaternion formed by Hamilton product of q1 and q2.
        Expects two arrays array of shape (4, )
        Returns an array of shape (4, ).
        Hint: The Hamiltonian product is not commutative. Ensure your operands are correctly placed.
    """
    raise NotImplementedError


def conjugate(q):
    """
        Calculate the quaternion conjugate. For a unit quaternion, this is the same as the inverse.
        Expects an array of shape (4, )
        Returns an array of shape (4, ).
    """
    raise NotImplementedError


def rotate(q, v):
    """
        Rotate vector v about the rotation described by a unit quaternion q.
        Expects an array of shape (4, ) for q and an array of shape (3, ) for v,
        Returns an array of shape (3, ).
    """
    raise NotImplementedError


def relative_angle(q1, q2):
    """
        Calculate the angle between unit quaternions q1 and q2.
        Expects two arrays of shape (4, )
        Returns a non-negative value.
    """
    raise NotImplementedError

def quaternion_to_matrix(q):
    """
        Convert a unit quaternion q to rotation matrix.
        Expects an array of shape (4, )
        Returns an array of shape (3, 3).
    """
    raise NotImplementedError

def matrix_to_quaternion(matrix):
    """
        Convert a rotation matrix to unit quaternion q.
        Expects an array of shape (3, 3)
        Returns an array of shape (4, ).
        Note: The rotation matrix is guaranteed to be a pure rotation, i.e. special orthogonal.
    """
    raise NotImplementedError

def quaternion_to_rotvec(q):
    """
        Convert a unit quaternion to axis-angle rotation (aka rotation vector or exponential coordinate).
        Expects an array of shape (4, ),
        Returns an array of shape (3, ).
    """
    raise NotImplementedError

def rotvec_to_quaternion(rv):
    """
        Convert an axis-angle rotation (aka rotation vector or exponential coordinate) to unit quaternion.
        Expects an array of shape (3, ),
        Returns an array of shape (4, ).
    """
    raise NotImplementedError

def rotvec_to_matrix(rv):
    """
        Convert an axis-angle rotation (aka rotation vector or exponential coordinate) to rotation matrix.
        Expects an array of shape (3, )
        Returns an array of shape (3, 3).
    """
    raise NotImplementedError

def matrix_to_rotvec(mat):
    """
        Convert a rotation matrix to axis-angle rotation (aka rotation vector or exponential coordinate).
        Expects an array of shape (3, 3)
        Returns an array of shape (3, ).
    """
    raise NotImplementedError

def generate_random_quaternion():
    """
        Uniformly sample a unit quaternion in SO(3).
        Returns an array of shape (4, ).
    """
    raise NotImplementedError

def interpolate_quaternions(q1, q2, ratio):
    """
        Find a valid quaternion rotation at a specified distance along the minor arc of a great circle passing through any two existing quaternion endpoints lying on the unit radius hypersphere.
        ratio: interpolation parameter between 0 and 1. This describes the linear placement position of the result along the arc between endpoints; 0 being at q1 and 1 being at q2.
        Expects two arrays of shape (4, ) and a ratio parameter.
        Returns an array of shape (4, ).
    """
    raise NotImplementedError














