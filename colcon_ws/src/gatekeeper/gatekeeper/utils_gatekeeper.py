import numpy as np
from timeit import default_timer as timer
from numba import njit
from numpy.linalg import norm
from matplotlib import pyplot as plt

@njit(cache=True, fastmath=True)
def construct_nominal_trajectory_for_goal(start_pos, goal_pos, v=1.0, dt=0.01, tmax=2.0, order=3, tracking_r = 0.4):

    DIM = 3

    N = int(tmax / dt)

    pos_traj = np.zeros((N+1, DIM))
    pos_traj[0, :] = start_pos

    delta = goal_pos - start_pos
    L = norm(delta)
    unit_dir = delta / norm(delta)
    
    for i in range(N):
        # see how far to go now
        curr_d =  norm(goal_pos - pos_traj[i, :])
        
        if curr_d > tracking_r:
            # still a lot to go
            vel = v * unit_dir
            pos_traj[i+1, :] = pos_traj[i, :] + vel * dt 
        else:
            # now try to get to the target location exactly 
            pos_traj[i+1, :] = goal_pos

    # now construct the actual reference trajectory
    x_ref = np.zeros((N+1, order*DIM))
    u_ref = np.zeros((N, DIM))

    for j in range(DIM):
        x_ref[:, order*j] = pos_traj[:, j] # copy over the position
        for i in range(N):
            x_ref[i, order*j+1] = (pos_traj[i+1, j] - pos_traj[i, j]) / dt

        x_ref[N, order*j+1] = x_ref[N-1, order*j+1] #copy over the last speed

    # assume nominal acc, jerk = 0
    return x_ref, u_ref


@njit(cache=True, fastmath=True)
def traj_inside_sfc(pos_traj, sfc_A, sfc_b, r=0.0, ground_z_margin = -1.0):

    N = pos_traj.shape[0]

    # create modified sfcs
    n_ = np.empty_like(sfc_b)
    for i in range(len(n_)):
        n_[i] = norm(sfc_A[i,:])

    b_minus_r = sfc_b - r * n_
    b_minus_2r = sfc_b - 3 * r * n_

    B = np.kron(np.ones((N,1)), b_minus_r)
    B[-1, :] = b_minus_2r # make the last one use twice the radius
    # B[-1, :] = b_minus_r # make the last one use twice the radius

    # now check 
    res = sfc_A @ pos_traj.T > B.T
    if np.any(res):
        for i in range(N):
            if np.any(res[:, i]) and pos_traj[i, 2] > ground_z_margin:
                return False

    return True


if __name__ == "__main__":

    start_pos = np.zeros(3)
    goal_pos = np.zeros(3)
    goal_pos[0] = 1.0


    for i in range(3):
      start = timer()
      x_ref, u_ref = construct_nominal_trajectory(start_pos, goal_pos)
      end = timer()
      print(end - start) 

    plt.plot(x_ref[:, 0])
    plt.plot(x_ref[:, 1])
    plt.show()


    # ## now try the sfc
    # from  utils_sfc import SFC
    # sfc = SFC.from_cube(1.21)

    # # now check safety
    # for i in range(30):
    #   start = timer()
    #   suc = traj_inside_sfc(x_ref[:, [0,3,6]], sfc.A, sfc.b, 0.1)
    #   end = timer()
    #   print(end-start)




