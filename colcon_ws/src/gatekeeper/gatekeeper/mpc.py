# in this version, it receives a goal message, and simply publishes a trajectory that connects the current state to the target state
# this file works in ENU vicon/world frame
import time
import numpy as np
from numpy.linalg import norm

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel
from dasc_msgs.msg import DITrajectory
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from decomp_ros_msgs.msg import PolyhedronStamped
import builtin_interfaces.msg

from .utils_euler import euler2quat, quat2euler
from .utils_sfc import SFC
# from .utils_gatekeeper import construct_nominal_trajectory, traj_inside_sfc
from .utils_integrator import TripleIntegrator
from .utils_mpc import MPCPlanner


class MPCNode(Node):
        
    POS_INDS = [0,3,6]
    VEL_INDS = [1,4,7]
    ACC_INDS = [2,5,8]

    def __init__(self):

        super().__init__("mpc_node")

        ## PARAMS
        self.update_rate_hz = 50.0 # hz
        self.traj_rate_hz = 50.0 # hz
        self.traj_horizon_s = 0.5 # seconds
        self.drone_radius = 0.2 # meters
        self.tracking_radius = 0.1 # meters
        self.use_sim_time = self.get_parameter("use_sim_time").value

        self.get_logger().warn(f"USE_SIM_TIME: {self.use_sim_time}")
        
        ## VARS
        self.state_initialized = False
        self.goal_initialized = False
        self.sfc = None
        self.mpc = MPCPlanner(
                DT = 1.0 / self.traj_rate_hz,
                N = int(self.traj_horizon_s * self.traj_rate_hz),
                max_accel=20.0)
        
        ## PUBS
        # self.pub_traj_ = self.create_publisher(DITrajectory, 'committed_traj', 10)
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint", 1)

        ## SUBS
        self.sub_state_ = self.create_subscription( 
                VehicleLocalPosition,
                'px4_1/fmu/out/vehicle_local_position',
                self.state_callback,
                qos_profile_sensor_data)

        self.sub_goal_pose_ = self.create_subscription(
                PoseStamped, 
                "/goal_pose",
                self.goal_callback,
                1)

        self.sub_sfc_ = self.create_subscription(
                PolyhedronStamped,
                "/nvblox_node/sfc",
                self.sfc_callback,
                1)


        ## TIMERS
        self.timer_ = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info("Starting mpc")

    def state_callback(self, state_msg):

        if not (state_msg.xy_valid  and state_msg.z_valid  and state_msg.heading_good_for_control):
            return

        self.state = np.array([
            state_msg.y,
            state_msg.vy,
            state_msg.ay,
            state_msg.x,
            state_msg.vx,
            state_msg.ax,
            -state_msg.z,
            -state_msg.vz,
            -state_msg.az,
            np.pi/2 - state_msg.heading])

        self.state_initialized = True
        
        if self.use_sim_time:
            unix_time_ns = time.time_ns()
            lag_ns = unix_time_ns - (state_msg.timestamp_sample * 1000)
        else:
            time_ns = self.get_clock().now().nanoseconds # when using gazebo, the publish rate is attrocious
            lag_ns = time_ns - (state_msg.timestamp_sample*1000)

        now_ns  = self.get_clock().now().nanoseconds

        extra_lag_ms = 300.0

        msg_ns = now_ns - lag_ns - extra_lag_ms * 10**6

        shifted_s = int(msg_ns // 10**9)
        shifted_ns = int(msg_ns % 10**9)

        # self.get_logger().info(f"lag: {lag_ns/1e6} ms")

        # self.get_logger().info(f"unix::now { unix_time_ns } NOW: ${now_ns}, PX4: {state_msg.timestamp}, SAMPLE: {state_msg.timestamp_sample}")
        

        self.state_stamp = builtin_interfaces.msg.Time(sec = shifted_s, nanosec=shifted_ns)

    def goal_callback(self, goal_msg):
        
        if goal_msg.header.frame_id != "vicon/world":
            self.get_logger().warn(f"goal is in frame {goal_msg.header.frame_id}, not vicon/world")
            return

        self.goal_yaw = quat2euler(goal_msg.pose.orientation)[2]

        self.get_logger().info(f"goal yaw: {self.goal_yaw}")

        self.goal = np.array([
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
            0.5,
            self.goal_yaw 
            ])

        self.goal_initialized = True

        self.timer_callback()

    def sfc_callback(self, sfc_msg):

        # check that it is in the correct frame
        if sfc_msg.header.frame_id != "vicon/world":
            self.get_logger().warn(f"sfc is in frame {sfc_msg.header.frame_id}, not vicon/world")
            return

        # store it
        sfc = SFC.from_msg(sfc_msg.poly)
        self.sfc = sfc.shrink(radius=self.drone_radius)
        self.get_logger().info("stored sfc")
        return


    def get_nominal_trajectory(self, start_pos, goal_pos, vmax=1.0):

        dt = 1.0 / self.traj_rate_hz

        x_ref, u_ref = construct_nominal_trajectory(start_pos, goal_pos, v=vmax, dt=dt, tmax=self.traj_horizon_s)

        return x_ref, u_ref



    def gatekeeper(self, start_state, goal_state, vmax=1.0):

        pos0 = start_state[self.POS_INDS]
        vel0 = start_state[self.VEL_INDS]
        acc0 = start_state[self.ACC_INDS]
        goal_pos = goal_state[0:3] 

        # self.get_logger().info(f"GK: Start: {pos0}, Stop: {goal_pos}")

        x_ref, u_ref = self.get_nominal_trajectory(pos0, goal_pos, vmax=vmax)

        ## now construct the tree of solutions 
        self.integrator.set_state(pos0, vel0, acc0)
        self.integrator.set_reference_trajectory(x_ref, u_ref)
        sols = self.integrator.sim5late_tree(1.0 / self.branch_rate_hz)

        # check which we want to keep
        for sol in sols: # the one with the longest TS is first
            # check validity
            if self.is_valid(sol[0], sol[1]):
                return sol

        return None

    def is_valid(self, sol_x, sol_u):

        eps = 0.1
        
        N = sol_x.shape[0]

        # check terminal control input
        terminal_u = sol_u[-1]
        if norm(terminal_u) > eps:
            # print(f"TOO MUCH U: {terminal_u}")
            return False

        # check terminal velocity
        terminal_v = sol_x[-1, self.VEL_INDS] 
        if norm(terminal_v) > eps:
            # print(f"TOO FAST: {terminal_v}")
            return False

        # # now check that all the states are in the sfc:
        pos_traj = sol_x[:, self.POS_INDS]
        if self.sfc is not None:
            if not traj_inside_sfc(pos_traj, self.sfc.A, self.sfc.b, r=self.tracking_radius):
                return False

        return True

        
    def timer_callback(self):

        ## GUARDS
        if not (self.state_initialized and self.goal_initialized):
            self.get_logger().info("waiting for state or goal msgs...")
            return

        start_timer = time.time()

        # run mpc
        self.mpc.set_state(self.state[self.POS_INDS], self.state[self.VEL_INDS], self.state[self.ACC_INDS])
        self.mpc.set_target_location(self.goal) # only grabs the position states
        res = self.mpc.solve()

        mpc_solve_time = time.time() - start_timer
        

        if not res:
            self.get_logger().warn("MPC RESOLVE FAILED")
            assert False, "MPC FAILED TO SOLVE"
            return

        # now publish the some indices in the future as a setpoint
        SKIP_INDS = 1

        setpoint_x = self.mpc.sol_x[SKIP_INDS]
        setpoint_u = self.mpc.sol_u[SKIP_INDS]

        # construct the message
        # convert to FRD frame at the same time
        px4_msg = TrajectorySetpoint()
        px4_msg.raw_mode = False;
        px4_msg.position[1] = setpoint_x[0]
        px4_msg.position[0] = setpoint_x[2]
        px4_msg.position[2] = - setpoint_x[4]

        px4_msg.velocity[1] = setpoint_x[1]
        px4_msg.velocity[0] = setpoint_x[3]
        px4_msg.velocity[2] = - setpoint_x[5]


        px4_msg.acceleration[1] = setpoint_u[0]
        px4_msg.acceleration[0] = setpoint_u[1]
        px4_msg.acceleration[2] = - setpoint_u[2]

        for i in range(3):
            px4_msg.jerk[i] = 0

        px4_msg.yaw = np.pi/2 - self.goal_yaw

        self.pub_px4_.publish(px4_msg)
        
        end_time = time.time() - start_timer
        self.get_logger().info(f"solve: {mpc_solve_time*1000:.3f} ms, total:  {end_time*1000:.3f} ms")


def main(args = None):

    rclpy.init(args=args)

    node = MPCNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
