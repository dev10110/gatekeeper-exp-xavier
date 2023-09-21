import time
import numpy as np
from numpy.linalg import norm

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DITrajectory
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from decomp_ros_msgs.msg import PolyhedronStamped
from nav_msgs.msg import Path
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker

from .utils_euler import quat2yaw, yaw2quat
from .utils_sfc import SFC
from .utils_gatekeeper  import construct_nominal_trajectory_for_goal, traj_inside_sfc
from .utils_integrator import Integrator
from .utils_mpc import MPCPlanner
from .utils_interpolator import Interpolator


class Gatekeeper(Node):

    def __init__(self):

        super().__init__("gatekeeper")

        ## PARAMS
        self.update_rate_hz = 20.0 # hz
        self.traj_rate_hz = 50.0 # hz
        self.branch_rate_hz = 5.0 # hz 
        self.traj_horizon_s = 2.0 # seconds
        self.drone_radius = 0.15 # meters
        self.tracking_radius = 0.10  # meters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.declare_parameter('fly_at_z_m', 1.0)
        self.fly_at_z = self.get_parameter("fly_at_z_m").value

        self.use_gk_mode = True # set true to use gatekeeper to do the filtering
                                   # set false to use mpc to do the filtering

        self.ORDER = 2
        self.DIM = 3

        self.POS_INDS = [self.ORDER * j + 0 for j in range(self.DIM)]
        self.VEL_INDS = [self.ORDER * j + 1 for j in range(self.DIM)] if (self.ORDER >= 2)  else []
        self.ACC_INDS = [self.ORDER * j + 2 for j in range(self.DIM)] if (self.ORDER >= 3) else []
        
        ## VARS
        self.state = None
        self.goal = None
        self.sfc = None
        self.nom_traj_msg = None
        self.interpolator = None
        self.integrator = Integrator(1.0/self.traj_rate_hz, order=self.ORDER) # double integrator
        self.mpc = MPCPlanner(N = int(self.traj_horizon_s * self.traj_rate_hz), DT=(1.0 / self.traj_rate_hz), max_accel=50.0)
        
        ## PUBS
        self.pub_traj_ = self.create_publisher(DITrajectory, 'committed_traj', 10)
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint", 10)
        self.pub_virtual_obs_ = self.create_publisher(PointStamped, "virtual_obstacles", 10)
        self.pub_chebyshev_ = self.create_publisher(PointStamped, "/nvblox_node/sfc/chebyshev_center", 10)
        self.pub_interpolated_nom_ = self.create_publisher(PoseArray, "nominal_traj/interpolated", 10)
        self.pub_committed_viz_ = self.create_publisher(PoseArray, "committed_traj/viz", 10)
        self.pub_committed_viz2_ = self.create_publisher(Marker, "committed_traj/viz2", 10)
        self.pub_timer_gk_ = self.create_publisher(builtin_interfaces.msg.Duration, "/diagnostics/timing/gatekeeper", 10)
        self.pub_timer_gk_total_ = self.create_publisher(builtin_interfaces.msg.Duration, "/diagnostics/timing/gatekeeper_total", 10)
        self.pub_timer_mpc_ = self.create_publisher(builtin_interfaces.msg.Duration, "/diagnostics/timing/mpc", 10)
        self.pub_timer_mpc_total_ = self.create_publisher(builtin_interfaces.msg.Duration, "/diagnostics/timing/mpc_total", 10)

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
        self.sub_nom_traj_ = self.create_subscription(
                  DITrajectory, 
                  "nominal_traj",
                  self.traj_callback,
                  1)

        self.sub_sfc_ = self.create_subscription(
                PolyhedronStamped,
                "/nvblox_node/sfc",
                self.sfc_callback,
                1)


        ## TIMERS
        self.timer_ = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info("Starting gatekeeper")

    def state_callback(self, state_msg):

        if not (state_msg.xy_valid  and state_msg.z_valid  and state_msg.heading_good_for_control):
            return

        if (self.ORDER == 2):
          self.state = np.array([
              state_msg.y,
              state_msg.vy,
              state_msg.x,
              state_msg.vx,
              -state_msg.z,
              -state_msg.vz])
        
        elif (self.ORDER == 3):
          self.state = np.array([
              state_msg.y,
              state_msg.vy,
              state_msg.ay,
              state_msg.x,
              state_msg.vx,
              state_msg.ax,
              -state_msg.z,
              -state_msg.vz,
              -state_msg.az])
        else:
            assert False

        self.state_yaw = np.pi/2 - state_msg.heading # in ENU coords

        self.state_initialized = True
        
        if self.use_sim_time:
            unix_time_ns = time.time_ns()
            lag_ns = unix_time_ns - (state_msg.timestamp_sample * 1000)
        else:
            time_ns = self.get_clock().now().nanoseconds # when using gazebo, the publish rate is attrocious
            lag_ns = time_ns - (state_msg.timestamp_sample*1000)

        now_ns  = self.get_clock().now().nanoseconds

        extra_lag_ms = 0*1000.0/self.traj_rate_hz # i.e. a skip_idx = 1

        msg_ns = now_ns - lag_ns - extra_lag_ms * 10**6

        shifted_s = int(msg_ns // 10**9)
        shifted_ns = int(msg_ns % 10**9)

        self.state_stamp = builtin_interfaces.msg.Time(sec = shifted_s, nanosec=shifted_ns)

    def goal_callback(self, goal_msg):
        
        if goal_msg.header.frame_id != "vicon/world":
            self.get_logger().warn(f"goal is in frame {goal_msg.header.frame_id}, not vicon/world")
            return

        self.goal_yaw = quat2yaw(goal_msg.pose.orientation)

        self.goal = np.array([
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
            self.fly_at_z,
            self.goal_yaw
            ])
    
    def traj_callback(self, traj_msg):

        # GUARDS 
        if traj_msg.header.frame_id != "vicon/world":
            self.get_logger().warn(f"traj is in frame {traj_msg.header.frame_id}, not vicon/world")
            return

        # Save it
        self.nom_traj_msg = traj_msg

        # interpolate it
        msg_N = len(traj_msg.poses)
        msg_t0 = traj_msg.header.stamp.sec + 1e-9 * traj_msg.header.stamp.nanosec
        msg_dt = traj_msg.dt 
        msg_ts = [msg_t0 + i * msg_dt for i in range(msg_N)]

        self.interpolator = Interpolator(msg_ts, traj_msg.poses, order=self.ORDER)

        return


    def sfc_callback(self, sfc_msg):

        # check that it is in the correct frame
        if sfc_msg.header.frame_id != "vicon/world":
            self.get_logger().warn(f"sfc is in frame {sfc_msg.header.frame_id}, not vicon/world")
            return

        # store it
        sfc = SFC.from_msg(sfc_msg.poly)
        self.sfc = sfc.shrink(radius=self.drone_radius)

        # compute the chebyshev center and publish it
        suc, cheby = self.sfc.chebyshev();
        if (suc):
          self.publish_chebyshev_center(sfc_msg.header, cheby)

        return

    def publish_chebyshev_center(self, header, center):

        msg = PointStamped()
        msg.header = header
        msg.point.x = center[0]
        msg.point.y = center[1]
        msg.point.z = center[2]
        
        self.pub_chebyshev_.publish(msg)

    def load_nominal_trajectory(self):

        # if self.use_goal_mode:
        #     return self.load_nominal_trajectory_goal_mode()
        # else:
            return self.load_nominal_trajectory_traj_mode()
        

    # def load_nominal_trajectory_goal_mode(self, vmax=1.0):

    #     dt = 1.0 / self.traj_rate_hz

    #     goal_pos = self.goal[0:3]
    #     goal_yaw = self.goal[3]

    #     pos0 = self.state[self.POS_INDS]
    #     vel0 = self.state[self.VEL_INDS] if (self.ORDER >= 2) else None
    #     acc0 = self.state[self.ACC_INDS] if (self.ORDER >= 3) else None

    #     use_MPC = False

    #     if use_MPC:
    #         self.mpc.set_state(pos0, vel0, acc0)
    #         self.mpc.set_target_location(goal_pos)
    #         self.mpc.solve()
    #         self.x_ref = np.vstack(self.mpc.sol_x)
    #         self.u_ref = np.vstack(self.mpc.sol_u)

    #         self.yaw_ref = goal_yaw * np.ones(self.x_ref.shape[0])
    #         return
    #     else:
    #         self.x_ref, self.u_ref = construct_nominal_trajectory_for_goal(
    #             pos0, goal_pos, v=vmax, dt=dt, tmax=self.traj_horizon_s, order=self.ORDER)
    #         self.yaw_ref = goal_yaw * np.ones(self.x_ref.shape[0])
    #         return

    def load_nominal_trajectory_traj_mode(self):

        t0 = self.state_stamp.sec + 1e-9 * self.state_stamp.nanosec
        dt = 1.0 / self.traj_rate_hz 

        # construct the nominal trajectory by interpolation
        ts = np.arange(start=t0, stop=t0 + self.traj_horizon_s, step = dt)

        # interpolate
        self.x_ref, self.yaw_ref, self.u_ref = self.interpolator.query(ts)

        self.publish_interpolated_nominal()

        return

    def publish_interpolated_nominal(self):

        msg = PoseArray()
        msg.header.frame_id = "vicon/world"
        msg.header.stamp = self.state_stamp
        N = self.x_ref.shape[0]
        for i in range(N):

            pose = Pose()
            pose.position.x = self.x_ref[i, self.POS_INDS[0]]
            pose.position.y = self.x_ref[i, self.POS_INDS[1]]
            pose.position.z = self.x_ref[i, self.POS_INDS[2]]
            pose.orientation = yaw2quat(self.yaw_ref[i])

            msg.poses.append(pose)

        self.pub_interpolated_nom_.publish(msg)


    def gatekeeper(self):

        pos0 = self.state[self.POS_INDS]
        vel0 = self.state[self.VEL_INDS] 
        acc0 = self.state[self.ACC_INDS] if (self.ORDER >= 3) else None
        yaw0 = self.state_yaw

        ## now construct the tree of solutions 
        self.integrator.set_state(pos0, vel0, acc0, yaw0)
        self.integrator.set_reference_trajectory(self.x_ref, self.yaw_ref, self.u_ref)
        sols = self.integrator.simulate_tree(1.0 / self.branch_rate_hz)

        # self.get_logger().info(f"WITH {len(sols)} solutions!")

        # check which we want to keep
        for i, sol in enumerate(sols): # the one with the longest TS is first
            
            # check validity
            if self.is_valid(sol[0], sol[1], sol[2])[0]:
                # self.get_logger().info(f"sol {i} is valid, terminal @ {sol[0][-1, self.POS_INDS]}")

                # set z to be 1.0
                # sol[0][:,self.POS_INDS[2]]  = 1.0
                # sol[0][:, self.VEL_INDS[2]] = 0.0
                # sol[2][:, 2] = 0.0
                return sol

        # if i am here, none of the solutions were valid.

        # check why the last one was invalid
        isvalid, reason = self.is_valid(sols[-1][0], sols[-1][1], sols[-1][2])
        
        if (not isvalid) and (reason == "sfc") and (np.linalg.norm(vel0) < 0.05) and (abs(pos0[2] - self.fly_at_z) < 0.2):
          # see which is the first nominals that is not in the sfc
          for i in range(self.x_ref.shape[0]):
                # but check that the nominal is in the correct height range:
                pos = self.x_ref[i, self.POS_INDS]
                if (abs(pos[2] - self.fly_at_z) < 0.2) and (not self.sfc.contains(pos)):
                    if (i < (self.x_ref.shape[0]-1)):
                        self.publish_virtual_obstacle(pos)
                        return None

    def replan_mpc(self):

        pos0 = self.state[self.POS_INDS]
        vel0 = self.state[self.VEL_INDS] 
        acc0 = self.state[self.ACC_INDS] if (self.ORDER >= 3) else None
        yaw0 = self.state_yaw

        self.mpc.set_state(pos0, vel0, acc0)
        x_ref = np.vstack([self.x_ref, self.x_ref[-1, :]]) # to be N+1 states
        self.mpc.set_ref_trajectory(x_ref, self.u_ref)

        self.mpc.set_safe_polyhedron(self.sfc.A, self.sfc.b, r=self.tracking_radius)

        # solve mpc problem
        suc = self.mpc.solve()

        if suc:
            # replanning was successful
            x_sol = np.vstack(self.mpc.sol_x)
            u_sol = np.vstack(self.mpc.sol_u)

            return x_sol, self.yaw_ref, u_sol
        
        else:
            # publish virtual obstacle
            if (abs(pos0[2] - self.fly_at_z) <= 0.2) and np.linalg.norm(vel0) < 0.05:
                # see which is the first nominals that is not in the sfc
                last_in_sfc_ind = None
                for i in range(self.x_ref.shape[0]):
                      # but check that the nominal is in the correct height range:
                      pos = self.x_ref[i, self.POS_INDS]
                      if (abs(pos[2] - self.fly_at_z) <= 0.2) and (not self.sfc.contains(pos)):
                          last_in_sfc_ind = i
                          break
                      else:
                          last_in_sfc_ind = i

                if (last_in_sfc_ind is not None and last_in_sfc_ind < (self.x_ref.shape[0]-1)):
                    self.publish_virtual_obstacle(self.x_ref[last_in_sfc_ind, self.POS_INDS])

    def publish_virtual_obstacle(self, p):
        msg = PointStamped()
        msg.header.frame_id = "vicon/world"
        msg.header.stamp = self.get_clock().now().to_msg();

        msg.point.x = p[0]
        msg.point.y = p[1]
        msg.point.z = p[2]

        self.get_logger().warn(f"pubbing virt obstacle at ({p[0]}, {p[1]}, {p[2]})")

        self.pub_virtual_obs_.publish(msg);


    def is_valid(self, sol_x, sol_yaw, sol_u):

        eps = 0.1
        
        N = sol_x.shape[0]

        # check terminal control input
        terminal_u = sol_u[-1]
        if norm(terminal_u) > eps:
            #self.get_logger().warn(f"TOO MUCH U: {terminal_u}")
            return False, "u"

        # check terminal velocity
        terminal_v = sol_x[-1, self.VEL_INDS] 
        if norm(terminal_v) > eps:
            # self.get_logger().warn(f"TOO FAST: {terminal_v}")
            return False, "v"

        # # now check that all the states are in the sfc:
        if self.sfc is not None:
            pos_traj = sol_x[:, self.POS_INDS]
            if not traj_inside_sfc(pos_traj, self.sfc.A, self.sfc.b, r=self.tracking_radius, ground_z_margin = 0.3):
                return False, "sfc"

        return True, "passed"

        
    def timer_callback(self):


        ## GUARDS
        if (self.state is None):
            self.get_logger().info("waiting for state msgs...")
            return

        if (self.goal is None):
            self.get_logger().info("waiting for goal msgs...")
            return

        if (self.nom_traj_msg is None):
            self.get_logger().info("waiting for nom traj msgs...")
            return
        
        start_time_ns = time.perf_counter_ns()

        self.load_nominal_trajectory()
        
        if self.use_gk_mode:
            sol = self.gatekeeper() 
        else:
            sol = self.replan_mpc()

        solve_time_ns = (time.perf_counter_ns() - start_time_ns)

        if sol is None:
            self.get_logger().warn("unable to find a safe traj")
            self.get_logger().info(f"solve: {solve_time_ns*1e-6:.3f} ms")
            return

        sol_x, sol_yaw, sol_u = sol

        self.publish_traj(sol_x, sol_yaw, sol_u)

        total_time_ns = time.perf_counter_ns() - start_time_ns
        self.get_logger().info(f"solve: {solve_time_ns*1e-6:.3f} ms, total: {total_time_ns*1e-6:.3f} ms")

        ## publish timers
        sec2ns = 1e9
        timer_msg_solve = builtin_interfaces.msg.Duration()
        timer_msg_solve.sec = int( solve_time_ns // sec2ns)
        timer_msg_solve.nanosec = int( solve_time_ns % sec2ns)

        timer_msg_total = builtin_interfaces.msg.Duration()
        timer_msg_total.sec = int (total_time_ns // sec2ns) 
        timer_msg_total.nanosec = int(total_time_ns % sec2ns)

        if self.use_gk_mode:
            self.pub_timer_gk_.publish(timer_msg_solve)
            self.pub_timer_gk_total_.publish(timer_msg_total)
        else:
            self.pub_timer_mpc_.publish(timer_msg_solve)
            self.pub_timer_mpc_total_.publish(timer_msg_total)


    def publish_traj(self, sol_x, sol_yaw, sol_u):
        

        msg = DITrajectory()
        msg.header.stamp = self.state_stamp
        msg.header.frame_id = "vicon/world"
        msg.dt = 1.0 / self.traj_rate_hz

        N = sol_x.shape[0] - 1
        # print(f"sol_x: {sol_x.shape}, sol_yaw: {sol_yaw.shape}, sol_u: {sol_u.shape}")

        for i in range(N):
            state = sol_x[i,:]
            u = sol_u[i,:] 

            pose = Pose()
            pose.position.x = state[self.POS_INDS[0]]
            pose.position.y = state[self.POS_INDS[1]]
            pose.position.z = state[self.POS_INDS[2]]
            pose.orientation = yaw2quat(sol_yaw[i])
            msg.poses.append(pose)

            twist = Twist()
            twist.linear.x = state[self.VEL_INDS[0]]
            twist.linear.y = state[self.VEL_INDS[1]]
            twist.linear.z = state[self.VEL_INDS[2]]
            msg.twists.append(twist)

            accel = Accel()
            if (self.ORDER == 2):
                accel.linear.x = u[0]
                accel.linear.y = u[1]
                accel.linear.z = u[2]

            elif (self.ORDER ==3):
                accel.linear.x = state[self.ACC_INDS[0]]
                accel.linear.y = state[self.ACC_INDS[1]]
                accel.linear.z = state[self.ACC_INDS[2]]
            else:
                assert False

            msg.accelerations.append(accel)

        self.pub_traj_.publish(msg)

        ## now create viz msg
        viz_msg = PoseArray()
        viz_msg.header = msg.header
        viz_msg.poses = msg.poses
        self.pub_committed_viz_.publish(viz_msg)

def main(args = None):

    rclpy.init(args=args)

    node = Gatekeeper()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
