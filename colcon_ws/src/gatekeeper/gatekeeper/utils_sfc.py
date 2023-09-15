import numpy as np
from decomp_ros_msgs.msg import Polyhedron
import osqp
from scipy import sparse
from numpy.linalg import norm


def to_numpy(msg):
    return np.array([msg.x, msg.y, msg.z])

class SFC:

    """ 
    define an sfc of the form { x: A x <= b }
    """
    def __init__(self, A, b):

        self.A = A
        self.b = b

    @classmethod
    def from_msg(cls, msg):
        N = len(msg.ps)

        A = np.zeros([N, 3])
        b = np.zeros(N)

        for i in range(N):
            n = to_numpy(msg.ns[i])
            p = to_numpy(msg.ps[i])
            A[i, :] = n
            b[i] = np.dot(n, p)

        return cls(A, b)

    @classmethod
    def from_cube(cls, L):
        N = 6 # faces of a cube
        A = np.vstack([np.eye(3), -np.eye(3)])
        b = L*np.ones(N)

        return cls(A, b)

        


    def shrink(self, radius):

        b = self.b - radius * norm(self.A, axis=1)

        return SFC(self.A, b)


    """
    returns true if x is inside the SFC
    """
    def contains(self, x, radius = 0):
        
        if radius != 0:
            shrunk_sfc = self.shrink(radius)
            return shrunk_sfc.contains(x)
        
        return np.all(self.A @ x <= self.b)

    """
    projects a point onto the closest point that is inside the polyhedron

    uses OSQP to solve the problem
    argmin || x - p ||^2 st. Ax <= b

    """
    def project(self, p, radius = 0):
        
        if radius != 0:
            shrunk_sfc = self.shrink(radius)
            return shrunk_sfc.project(p)

        # argmin p'p -2 x'p + x'x st. Ap <= b
        # argmin 0.5 p'p - x' p  s.t. Ap <= b 

        P = sparse.csc_matrix(np.eye(3))
        q = - p
        A = sparse.csc_matrix(self.A)
        l = -np.inf * np.ones(np.shape(A)[0])
        u = self.b

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        prob.setup(P, q, A, l, u, verbose=False)

        # Solve problem
        res = prob.solve()

        suc = res.info.status == "solved"

        return suc, res.x

    """
    returns the chebyshev center of a polyhedron
    see https://web.stanford.edu/class/ee364a/lectures/problems.pdf
    """
    def chebyshev(self):

        # minimize_{x, r} -r
        # st. a_i @ x + r norm(a_i) <= b_i 

        P = sparse.csc_matrix(np.zeros((4,4)))
        q = np.array([0,0,0,-1.0])

        N = np.shape(self.A)[0]

        normA = np.reshape(np.linalg.norm(self.A, axis=1), (N, 1))
        A = sparse.csc_matrix(np.hstack([self.A, normA]))
        l = -np.inf * np.ones(N)
        u = self.b

        prob = osqp.OSQP()
        prob.setup(P, q, A, l, u, verbose=False)

        res = prob.solve()

        suc = res.info.status == "solved"

        return suc, res.x[0:3]











