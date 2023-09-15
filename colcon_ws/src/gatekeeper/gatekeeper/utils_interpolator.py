import numpy as np
import scipy as sp
from scipy.interpolate import interp1d, CubicSpline, UnivariateSpline
from .utils_euler import quat2yaw, yaw2quat
from scipy.spatial.transform import Rotation, Slerp
from geometry_msgs.msg import Quaternion


def spquat2yaw(r):

    q = Quaternion() # geometry_msgs/msg/Quaternion
    q.x = r[0]
    q.y = r[1]
    q.z = r[2]
    q.w = r[3]

    return quat2yaw(q) # float

class Interpolator: 

    def __init__(self, msg_ts, poses, order=2):

        self.msg_ts = msg_ts
        self.poses = poses
        self.ORDER = order
        self.DIM = 3
        self.DEGREE = 3

        self.construct_interpolants()


    def construct_interpolants(self):

        N = len(self.poses)
        self.pos = np.zeros((N, 3))
        self.pos[:, 0] = [p.position.x for p in self.poses]
        self.pos[:, 1] = [p.position.y for p in self.poses]
        self.pos[:, 2] = [p.position.z for p in self.poses]

        # store all the quaternions into a numpy array
        self.quats = np.zeros((N, 4))
        for i in range(N):
            p = self.poses[i]
            self.quats[i, :] = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

        self.rots = Rotation.from_quat(self.quats) # create a single rotation object from the list of quaternions

        if N <= 2:
            self.DEGREE = 1
        elif N <= 3:
            self.DEGREE = 2
        else:
            self.DEGREE = 3

        self.interp_pos = [UnivariateSpline(self.msg_ts, self.pos[:, i], k=self.DEGREE, s=0, ext="const") for i in range(3)]
        
        self.interp_rots = Slerp(self.msg_ts, self.rots)

    def query(self, ts):

        N = len(ts)
        
        sol_x = np.zeros((N, self.ORDER * self.DIM))
        sol_u  =  np.zeros((N, self.DIM))

        POS_INDS = [self.ORDER * i + 0 for i in range(self.DIM)]
        VEL_INDS = [self.ORDER * i + 1 for i in range(self.DIM)]
        ACC_INDS = [self.ORDER * i + 2 for i in range(self.DIM)] if self.ORDER == 3 else []

        for i in range(self.DIM):
            sol_x[:, POS_INDS[i]] = self.interp_pos[i](ts, nu=0, ext="const")
            sol_x[:, VEL_INDS[i]] = self.interp_pos[i](ts, nu=1, ext="zeros")
            if (self.ORDER == 3):
                sol_x[:, ACC_INDS[i]] = 0.0 if self.DEGREE < 2 else self.interp_pos[i](ts, nu=2, ext="zeros")
                sol_u[:, i] = 0.0 if self.DEGREE < 2 else self.interp_pos[i](ts, nu=3, ext="zeros")
            elif (self.ORDER == 2):
                sol_u[:, i] = 0.0 if self.DEGREE < 2 else self.interp_pos[i](ts, nu=2, ext="zeros")
            else: 
                assert False, "self.ORDER must be 2 or 3"

        # now interpolate yaws
        clipped_ts = np.clip(ts, self.msg_ts[0], self.msg_ts[-1]) # automatically force inbounds
        sol_rots = self.interp_rots(clipped_ts).as_quat() # convert to matrix of quaternions
        sol_yaw = np.apply_along_axis(spquat2yaw, 1, sol_rots)
        # surely there must be a better way to do this?

        return sol_x, sol_yaw, sol_u 



if __name__ == "__main__":

    import matplotlib.pyplot as plt
    from geometry_msgs.msg import Pose

    # test
    msg_ts = np.linspace(0.0, 5.0,  num=20) # i.e. a dt = 0.1
    # xs = 0.2 * msg_ts + 0.1 * np.random.randn(len(msg_ts)) + np.power(msg_ts, 2)
    xs = np.power(msg_ts, 2)
    ys = 0.0 * msg_ts
    zs = 1.0 + 0.0 * msg_ts
    yaws = np.pi/2  +  1.0 * msg_ts

    # construct poses
    poses = []
    for i in range(len(msg_ts)):
        p = Pose()
        p.position.x = xs[i]
        p.position.y = ys[i]
        p.position.z = zs[i]
        p.orientation = yaw2quat(yaws[i])
        poses.append(p)


    # construct interpolator
    intp = Interpolator(msg_ts, poses)

    # interpolate
    ts = np.arange(start = -1.0, stop = 6.0, step = 0.01)
    sol_xs, sol_yaws, sol_us = intp.query(ts)

    # plt.plot(msg_ts, xs, "x")
    # plt.plot(ts, sol_xs[:, 0])

    # plt.plot(ts, sol_xs[:, 1])
    # plt.plot(ts, sol_us[:, 0])
    
    plt.plot(msg_ts, yaws, "x")
    plt.plot(ts, sol_yaws)

    plt.show()








