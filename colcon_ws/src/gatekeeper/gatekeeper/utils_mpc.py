import time
import osqp
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt


## Based off the implementation suggested at
## https://osqp.org/docs/examples/mpc.html
## uses the same variable names as in that doc

DIM = 3

class MPCPlanner: 


    def __init__(self,
            N=20,  # planning horizon
            DT=0.1, # in seconds
            max_accel = 3.0 # m/s^2
            ):

        ## save the params
        self.N = N
        self.DT = DT
        self.max_accel = max_accel

        ## dynamics
        self.create_dynamics_matrices()
        
        # initial goal and x0
        self.xr = np.ones(2*DIM)
        self.x0 = np.zeros(2*DIM)

        # initialize sfc_A, sfc_b
        # constraint is of the form sfc_A @ x[pos_inds] <= sfc_b 

        # sfc_A = np.array([[0.0, 0, 1.0]])
        # sfc_b = np.array([0]) # default initializer just so there is something to work with
        # self.set_safe_polyhedron(sfc_A, sfc_b)
        self.sfc_A = None
        self.sfc_b = None

    def create_dynamics_matrices(self):
        A = np.array([
            [1, self.DT],
            [0, 1.0]])

        B = np.array([[0.5*self.DT**2],[self.DT]]) 

        I3 = np.eye(DIM)
        self.Ad = sparse.csc_matrix( np.kron(I3, A) )
        self.Bd = sparse.csc_matrix( np.kron(I3, B) )

        [nx, nu] = self.Bd.shape
        self.nx = nx # should be 6
        self.nu = nu # should be 3

    def set_state(self, pos, vel, acc):
        # acc is ignored? or the first acc is forced to be what is specified here?
        x0 = np.array([
            pos[0],
            vel[0],
            pos[1],
            vel[1],
            pos[2],
            vel[2]])

        self.set_x0(x0)

    def set_x0(self, x0):
        self.x0 = x0 
    
    def set_target_state(self, target_state):
        self.xr = target_state
        self.x_ref = np.kron(np.ones((self.N+1, 1)), self.xr).T
        self.u_ref = np.kron(np.ones((self.N, 1)), np.zeros(DIM)).T

    def set_target_location(self, target):
        target_state = np.zeros(2*DIM)
        for i in range(DIM):
            target_state[2*i] = target[i]
            target_state[2*i+1] = 0.0

        self.set_target_state(target_state)

    def set_ref_trajectory(self, x_ref, u_ref):
        
        # handle too short
        while (x_ref.shape[0] < self.N+1):
            x_ref = np.vstack([x_ref, x_ref[-1,:]])

        while (u_ref.shape[0] < self.N):
            u_ref = np.vstack([u_ref, np.zeros(self.nu)])

        # handle too long
        if (x_ref.shape[0] > self.N+1):
            x_ref = x_ref[:(self.N+1), :]
        
        if (u_ref.shape[0] > self.N):
            u_ref = u_ref[:(self.N), :]

        # now it should be the right length
        assert x_ref.shape == (self.N+1, self.nx),  f"Expected: {(self.N+1, self.nx)} Got: {x_ref.shape}"
        assert u_ref.shape == (self.N, self.nu),  f"Expected: {(self.N, self.nu)} Got: {u_ref.shape}"

        self.x_ref = x_ref.T
        self.u_ref = u_ref.T

    def set_safe_polyhedron(self, A, b, r=0):
        # r is the robustness margin

        N_sfc = A.shape[0]

        padded_A = np.insert(A, [1,2,3], np.zeros([N_sfc,3]), axis=1) # converts [a,b,c] to [a, 0, b, 0, c, 0]
        self.sfc_A = sparse.csc_matrix(padded_A)

        n_ = np.empty_like(b)
        for i in range(len(n_)):
            n_[i] = np.linalg.norm(A[i, :])
        b_minus_r = b - r * n_

        self.sfc_b = b_minus_r


    def construct_objective(self):


        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        N = self.N
        nx = self.nx
        nu = self.nu

        Q = 100 * np.eye(nx)
        # proritize z
        Q[4] *= 10.0
        # deprioritize vel tracking
        for i in range(DIM):
             Q[2*i + 1]  *= 0.1
        R = sparse.eye(nu)

        Q = sparse.csc_matrix(Q)
        P = sparse.block_diag(
                [
                    sparse.kron(sparse.eye(self.N+1), Q),
                    sparse.kron(sparse.eye(self.N), R)
                ], format='csc')


        # q = np.hstack([
        #     np.kron(np.ones(N), -Q@self.xr),
        #     -QN@self.xr,
        #     np.zeros(N*nu)])

        q = np.hstack([np.hstack((-Q @ self.x_ref).T), np.hstack((-R @ self.u_ref).T)])

        return P, q


    def construct_cons_dynamics(self):

        N = self.N
        nx = self.nx
        nu = self.nu

        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq

        return Aeq, leq, ueq


    def construct_cons_input(self):
        
        N = self.N
        nx = self.nx
        nu = self.nu

        umin = -self.max_accel;
        umax =  self.max_accel;

        Ax = sparse.csc_matrix(np.zeros([N*nu, (N+1)*nx]))
        Bu = sparse.eye(N*nu)

        Aineq = sparse.hstack([Ax, Bu])
        lineq = umin * np.ones(N*nu)
        uineq = umax * np.ones(N*nu)

        return Aineq, lineq, uineq


    def construct_cons_terminal(self):

        # forces the terminal speed to be 0
        
        N = self.N
        nx = self.nx
        nu = self.nu

        A = np.zeros((DIM, 2*DIM))
        for i in range(DIM):
            A[i, 2*i+1] = 1


        Aeq = sparse.hstack([
          sparse.csc_matrix(np.zeros([DIM, N*nx])),
          sparse.csc_matrix(A),
          sparse.csc_matrix(np.zeros([DIM, N*nu]))
          ])

        leq = np.zeros(DIM) 
        ueq = np.zeros(DIM)

        return Aeq, leq, ueq


    def construct_cons_sfc(self):

        N = self.N
        nx = self.nx
        nu = self.nu

        if self.sfc_A is None:

            A = np.zeros((0, (N+1)*nx + N*nu))
            l = np.zeros(0)
            u = np.zeros(0)
            return A, l, u


        N_sfc = self.sfc_A.shape[0] # number of constraints in the polyhedron

        # diagonalize it
        Ax = sparse.kron(sparse.eye(N+1), sparse.csc_matrix(self.sfc_A))
        Bu = sparse.csc_matrix(np.zeros([(N+1)*N_sfc, N*nu]))

        Aineq = sparse.hstack([Ax, Bu])

        sfc_l = (-np.inf) * np.ones(N_sfc)
        
        lineq = np.kron(np.ones(N+1), sfc_l)
        uineq = np.kron(np.ones(N+1), self.sfc_b)

        return Aineq, lineq, uineq


    def construct_constraints(self):


        # dynamics + initial condition
        cons_dyn = self.construct_cons_dynamics() # also has the initial state constraint
        
        # input constraints
        cons_input = self.construct_cons_input()
        
        # must come to a stop
        cons_terminal = self.construct_cons_terminal()
        
        # sfc constraints
        cons_sfc = self.construct_cons_sfc() 


        ## concatenate 

        cons = [cons_dyn,
                cons_input,
                cons_terminal,
                cons_sfc 
                ] 

        cons_A = sparse.vstack([c[0] for c in cons], format="csc")
        cons_l = np.hstack([c[1] for c in cons])
        cons_u = np.hstack([c[2] for c in cons])

        return cons_A, cons_l, cons_u


    def solve(self):

        ## Objective function
        P, q = self.construct_objective()

        ## constraints 
        A_cons, l_cons, u_cons = self.construct_constraints()
        # print(A_cons.shape, np.shape(l_cons), np.shape(u_cons))

        # Create an OSQP object
        prob = osqp.OSQP()
        
        # Setup workspace
        prob.setup(P, q, A_cons, l_cons, u_cons, verbose=False, polish=False, eps_abs=1e-6, eps_rel=1e-5)

        # Solve
        res = prob.solve()

        # Check solve status
        if res.info.status_val != 1:
            # print(res.info.status)
            return False

        # ok it solved optimally

        N = self.N
        nx = self.nx
        nu = self.nu

        # update the solution
        self.sol_x = [res.x[(nx*i):(nx*(i+1))] for i in range(N+1)]
        self.sol_u = [res.x[((N+1)*nx + nu*i):((N+1)*nx + nu*(i+1))] for i in range(N)]

        return True

    def print_solution(self):

        
        for i in range(self.N):
            print(f"{i}:: x={np.array2string(self.sol_x[i], precision=2, suppress_small=True)} u={np.array2string(self.sol_u[i], precision=2, suppress_small=True)}")
        
        i = self.N
        print(f"{i}:: x={np.array2string(self.sol_x[i], precision=2, suppress_small=True)}")

    def plot_solution(self):

        xs = [x[0] for x in self.sol_x]
        ys = [x[2] for x in self.sol_x]
        zs = [x[4] for x in self.sol_x]

        axs = [x[0] for x in self.sol_u]
        ays = [x[1] for x in self.sol_u]
        azs = [x[2] for x in self.sol_u]

        fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
        ax.stem(xs, ys, zs)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")

        ax.set_box_aspect([1,1,1])
        
        fig2, ax2 = plt.subplots()
        ax2.plot(xs, label="x")
        ax2.plot(ys, label="y")
        ax2.plot(zs, label="z")
        ax2.plot(self.x_ref[0,:], "--", label="x_ref")
        ax2.plot(self.x_ref[2,:], "--", label="y_ref")
        ax2.plot(self.x_ref[4,:], "--", label="z_ref")
        ax2.set_title("Pos")
        ax2.legend()
        
        fig3, ax3 = plt.subplots()
        ax3.plot(axs, label="x")
        ax3.plot(ays, label="y")
        ax3.plot(azs, label="z")
        ax3.set_title("acc")

        plt.show()

        return

if __name__ == "__main__":

    T_horizon = 2.0
    rate_hz = 50.0

    DT = 1.0 / rate_hz
    N = int(T_horizon / DT)

    mpc = MPCPlanner(
            N=N,  # planning horizon
            DT=DT, # in seconds
            max_accel = 50.0 # m/s^2
            )


    # mpc.set_state(np.array([0.25, 0.00,   0.25, 0.0,   0.25, 0.0  ]))
    mpc.set_state(pos=0.25*np.ones(3), vel= np.zeros(3), acc=np.zeros(3))
    # mpc.set_x0(np.ones(6))
    mpc.set_target_location(np.array([1,1,1]))

    # try setting a linear velocity target
    x_ref = np.zeros((N+1, 2*3))
    x_ref[0,:] = mpc.x0
    x_ref[0, 1] = 0.5 # target linear velocity on x
    x_ref[0, 3] = -0.25 # target linear velocity on x
    x_ref[0, 4] = 1.0 # target alt
    for i in range(N):
        x_ref[i+1, :] = x_ref[i, :]
        x_ref[i+1, 0] = x_ref[i, 0] + x_ref[i,1]* DT
        x_ref[i+1, 2] = x_ref[i, 2] + x_ref[i,3]* DT

    mpc.set_ref_trajectory(x_ref, np.zeros((N, 3)))
    

    start_time = time.time()
    sol = mpc.solve()
    end_time = time.time()

    mpc.print_solution()

    print(f"Time: {end_time - start_time} seconds")
    mpc.plot_solution()

