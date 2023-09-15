import numpy as np
from geometry_msgs.msg import Quaternion
from numba import njit

def euler2quat(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw (rad) to geometry_msgs/msg/Quaternion
    """

    _q = euler2quat_impl(roll, pitch, yaw)

    q = Quaternion()
    q.w = _q[0]
    q.x = _q[1]
    q.y = _q[2]
    q.z = _q[3]

    return q

def quat2euler(quaternion):
    """
    Converts geometry_msgs/msg/Quaternion  to roll, pitch yaw (rad)
    """
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w

    return quat2euler_impl(qw,qx,qy,qz)

def quat2yaw(quaternion):
    return quat2euler(quaternion)[2]

def yaw2quat(yaw):

    q = Quaternion()
    q.w = np.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = np.sin(yaw * 0.5)

    return q

@njit(cache=True)
def euler2quat_impl(roll, pitch, yaw):
    
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return (qw, qx, qy, qz)

@njit(cache=True)
def quat2euler_impl(qw, qx, qy, qz):
    
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

