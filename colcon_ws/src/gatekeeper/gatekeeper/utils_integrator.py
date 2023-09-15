import numpy as np
from timeit import default_timer as timer
import control # to get the dlqr function
import matplotlib.pyplot as plt
from numba import njit

factorial = np.math.factorial

@njit(cache=True, fastmath=True)
def fast_simulate(x0, x_ref, yaw_ref, u_ref, A, B, K):

    N, n = x_ref.shape
    Nu, m = u_ref.shape

    sol_x = np.zeros((N, n))
    sol_u = np.zeros((N, m))
    sol_yaw = yaw_ref.copy()

    sol_x[0,:] = x0

    for i in range(N-1):
        u = sol_u[i, :] + K @ (x_ref[i, :] - sol_x[i, :])
        sol_u[i, :] = u
        sol_x[i+1, :] = A @ sol_x[i, :] + B @ u

    return sol_x, sol_yaw, sol_u

@njit(cache=True, fastmath=True)
def fast_simulate_tracking_switched(x0, x_ref, yaw_ref, u_ref, A, B, K, dt, Tswitch):

    N, n = x_ref.shape
    Nu, m = u_ref.shape
    DIM = m
    ORDER = int(n / DIM)

    sol_x = np.zeros((N, n))
    sol_u = np.zeros((N, m))
    sol_yaw = np.zeros(N)

    sol_x[0,:] = x0
    sol_yaw[0] = yaw_ref[0]
    
    # propagate trajectory until ind
    ind = min(N-1, max(0, int(Tswitch / dt)))
    for i in range(ind):
        u = sol_u[i, :] + K @ (x_ref[i, :] - sol_x[i, :])
        sol_u[i, :] = u
        sol_x[i+1, :] = A @ sol_x[i, :] + B @ u
        sol_yaw[i+1] = yaw_ref[i+1]

    # determine the stop state
    x_stop = (sol_x[ind, :]).copy()
    yaw_stop = sol_yaw[ind]
    for j in range(n):
        if (j % ORDER  != 0):
            x_stop[j] = 0.0 # force it to stop here

    # propagate stopped trajectory
    u_stop_ref = np.zeros(DIM)
    for i in range(ind, N-1):
        e_state = x_stop - sol_x[i, :]

        if True:
            for j in range(DIM):
                e_state[ORDER*j + 0] = 0.0 # zeroed position error

        u = u_stop_ref + K @ e_state  # u_feedforward should be zero
        sol_u[i, :] = u
        sol_x[i+1, :] = A @ sol_x[i, :] + B @ u
        sol_yaw[i+1] = yaw_stop

    return sol_x, sol_yaw, sol_u
    
class Integrator:

    def __init__(self, dt, order):
        self.DIM = 3 # x,y,z
        self.ORDER = order # order = 3 for triple integrator, 2 for double integrator

        self.dt = dt
        self.n =  self.ORDER*self.DIM
        self.m = self.DIM

        self.construct_AB(self.dt)
        self.construct_K()

        self.x0 = np.zeros(self.n)
        self.x_ref = np.zeros((0, self.n)) # initialize the variables? 
        self.u_ref = np.zeros((0, self.m))
        self.yaw_ref = np.zeros(0)

        return

    def construct_AB(self, dt):

        A = np.zeros((self.ORDER,self.ORDER)) # 3 because it is a triple integrator
        for i in range(self.ORDER):
            for j in range(self.ORDER):
                k = j - i
                if k >= 0:
                  v = (self.dt) ** k / factorial(k)
                  A[i,j] = v

        B = np.zeros((self.ORDER,1))
        for i in range(self.ORDER):
            k = self.ORDER-i
            v = (self.dt) ** k / factorial(k)
            B[i,0] = v

        self.A = np.kron(np.eye(self.DIM), A)
        self.B = np.kron(np.eye(self.DIM), B)

    def construct_K(self):

        Q = np.kron(np.eye(self.DIM), np.diag([10**(self.ORDER-i) for i in range(self.ORDER)]))
        R = np.eye(self.m)

        self.K = control.dlqr(self.A, self.B, Q, R)[0]

    def set_state(self, pos, vel=None, acc=None, yaw=0.0):

        if vel is None:
            vel = np.zeros(self.DIM)

        if acc is None:
            acc = np.zeros(self.DIM)

        x0 = np.zeros(self.n)
        for i in range(self.DIM):
            x0[self.ORDER*i + 0] = pos[i]
            if self.ORDER >= 2:
              x0[self.ORDER*i + 1] = vel[i]
            if self.ORDER >= 3:
              x0[3*i + 2] = acc[i]

        return self.set_x0(x0, yaw)


    def set_x0(self, x0, yaw=0.0):

        assert len(x0) == self.n, f"x0 has incompatible dimension {len(x0)}, expected {self.n}" 
        
        self.x0 = x0
        self.yaw0 = yaw
        return

    def set_reference_trajectory(self, ref_x, ref_yaw, ref_u):
        N, n = ref_x.shape

        assert n == self.n, "incompatible dims!!"

        self.x_ref = ref_x
        self.u_ref = ref_u
        self.yaw_ref = ref_yaw

        return

    def rollout_zero(self, N):

        ref_x = np.zeros([N+1, self.n])
        ref_yaw = np.zeros(N+1)
        ref_u = np.zeros([N, self.m])

        ref_x[0,:] = self.x0
        ref_yaw[0] = self.yaw0

        return self.rollout(ref_x, ref_yaw, ref_u)

    def rollout(self, ref_x, ref_yaw, ref_u):

        Nx, n = ref_x.shape
        Ny    = ref_yaw.shape
        Nu, m = ref_u.shape

        assert Nx <= Nu + 1, "incompatible dims"
        assert n == self.n
        assert m == self.m
        assert Ny == Nx

        for i in range(Nx-1):
            ref_x[i+1, :] = self.A @ ref_x[i, :] + self.B @ ref_u[i,:]
            ref_yaw[i+1] = ref_yaw[i]

        return ref_x, ref_yaw, ref_u

    def simulate_tracking(self):

        # assumes the trajectory is of the form np.array(N, n)
        # where N is the number of timesteps, and n is the dimension of the state space
        # returns bool success, traj (N x n)  and feedforward control (N x m) where m is the dim of the control space
        # returns the tracked trajectory and the feedforward control input

        return fast_simulate(self.x0, self.x_ref, self.yaw_ref, self.u_ref, self.A, self.B, self.K)

    def simulate_tracking_switched(self, Tswitch):

        return fast_simulate_tracking_switched(self.x0, self.x_ref, self.yaw_ref, self.u_ref, self.A, self.B, self.K, self.dt, Tswitch)

    def simulate_tree(self, DT, tmax=2.0):

        N = self.x_ref.shape[0]
        T_nom = min(N * self.dt, tmax)

        ts = reversed(np.arange(start = DT, stop=T_nom, step=DT))


        sols = [ self.simulate_tracking_switched(t) for t in ts ]

        return sols


if __name__ == "__main__":

    dt = 0.02 # 50 hz
    order = 2 # double integrator
    horizon_s = 2.0 # seconds
    

    sim = Integrator(dt, order=2)

    # create a reference trajectory
    N = int(horizon_s / dt)

    # constant pos traj
    ref0_pos = np.array([1,0,0.])
    ref0_vel = np.array([0,0,0.])
    ref0_acc = np.array([0,0,0.])
    ref0_yaw = 0.0

    # constant speed traj
    if True:
      ref0_pos = np.array([0,0,0.])
      ref0_vel = np.array([1,0,0.])
      ref0_acc = np.array([0,0,0.])
      ref0_yaw = 0.0
    

    sim.set_state(ref0_pos, ref0_vel, ref0_acc, ref0_yaw)
    ref_x, ref_yaw, ref_u = sim.rollout_zero(N)

    pos0 = np.zeros(sim.DIM)
    vel0 = np.zeros(sim.DIM)
    acc0 = np.zeros(sim.DIM)
    yaw0 = np.pi

    sim.set_state(pos0, vel0, acc0, yaw0)
    sim.set_reference_trajectory(ref_x, ref_yaw, ref_u)


    #  # simulate
    #  for i in range(10):
    start = timer()
    sol_x, sol_yaw, sol_u = sim.simulate_tracking()
    end = timer()
    print(f"time: {end-start} s")
    #  
    #  print("\n\n\n")
    #  
    #  # simulate
    #  for i in range(10):
    #    start = timer()
    sol_x, sol_yaw, sol_u = sim.simulate_tracking_switched(1.0)
    #    end = timer()
    #    print(f"time: {end-start} s")


    start = timer()
    sols = sim.simulate_tree(0.2)
    end = timer()
    print(f"tree: {end-start} s")
    # start = timer()
    # sols = sim.simulate_tree(0.2)
    # end = timer()
    # print(f"tree: {end-start} s")



    ## PLOTS
    ts = [i * sim.dt for i in range(ref_x.shape[0])]

    fig = plt.figure()

    for i in range(order):
        ax = fig.add_subplot(2,2,i+1)
        ax.plot(ts, ref_x[:, i], "--",label="ref")
        ax.set_title(f"order {i}")

        for sol in sols:
          x_ref = sol[0]
          ax.plot(ts, x_ref[:, i])

    # next add the control input
    ax = fig.add_subplot(2,2,order+1)
    ax.plot(ts[:-1], ref_u[:,0], "--", label="ref")
    ax.set_title(f"u")
    for sol in sols:
        u_ref = sol[2]
        ax.plot(ts, u_ref[:,0])

    plt.show()

    # print(sol_x)
    # print(sol_u)

    # try to track the new trajectory
    # sim.set_reference_trajectory(sol_x, sol_u)
    # sol2_x, sol2_u = sim.simulate_tracking()

    # ts = [i * sim.dt for i in range(ref_x.shape[0])]


    # plt.plot(ts, [x[0] for x in ref_x], ".-", label="ref_x")
    # plt.plot(ts, [x[0] for x in sol_x], ".-", label="sol_x")
    # plt.plot(ts, [x[0] for x in sol2_x], ".-", label="sol2_x")
    # plt.show()



