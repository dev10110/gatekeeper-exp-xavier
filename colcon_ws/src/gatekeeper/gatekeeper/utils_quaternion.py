# this file extens the geometry_msgs/Quaternion to have some simple utilities
import numpy as np
from geometry_msgs.msg import Quaternion

# return the signed angle corresponding to this quaternion
def angle(self):
    l = np.sqrt(self.x**2 + self.y**2 + self.z**2)
    return 2.0 * np.arctan2(l, self.w)

def inverse(self):
    q = Quaternion()
    q.x = -self.x
    q.y = -self.y
    q.z = -self.z
    q.w = self.w
    return q

def quat_prod(self, other):
    # return the quaternion product self * other

    w0, x0, y0, z0 = self.w, self.x, self.y, self.z
    w1, x1, y1, z1 = other.w, other.x, other.y, other.z

    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    q = Quaternion()
    q.x = x
    q.y = y
    q.z = z
    q.w = w

    return q 

def angle_between(self, other):

    qi = inverse(other)
    qd = quat_prod(qi, self)

    return angle(qd)
