import rclpy
from rclpy.node import Node
import numpy as np

from .utils_euler import yaw2quat, quat2yaw

from dasc_msgs.msg import DITrajectory
from geometry_msgs.msg import Pose, Twist, Accel, PoseArray, PoseStamped, TransformStamped

import matplotlib.pyplot as plt


class NominalPlanner2(Node):

    def __init__(self):

        super().__init__("nominal_planner2")

        ## params

        ### general params
        self.path_type = "circle"
        self.horizon = 2.0 # seconds
        self.fly_at_z = 1.0 # meters
        self.period = 30.0 # seconds

        ### circle params
        self.circle_x0 = 0.0 # meters 
        self.circle_y0 = 0.0 # meters
        self.circle_r0  = 1.5 # meters
        self.circle_omega = 2*np.pi / self.period # seconds

        ### figure8 params
        self.figure8_x0 = 0.0 # meters 
        self.figure8_y0 = 0.0 # meters
        self.figure8_rx  = 1.0 # meters
        self.figure8_ry  = 0.5 # meters
        self.figure8_omega = 2*np.pi / self.period # seconds

        ## update params
        self.declare_parameter("fly_at_z_m", self.fly_at_z)
        self.fly_at_z = self.get_parameter("fly_at_z_m").value

        ## vars
        self.t0 = self.get_clock().now()
        self.started = False

        self.x = None # current state
        self.y = None
        self.z = None
        self.yaw = None

        # store the expected path
        if self.path_type == "circle":
            self.ref_theta, _, self.ref_path  = self.get_circle_path(0.0, self.period, 200)
        elif self.path_type == "figure8":
            self.ref_theta, _, self.ref_path = self.get_figure8_path(0.0, self.period, 200)
        
        self.ref = np.vstack([
            self.ref_path[1:4, :],  # xyz
            np.cos(self.ref_path[0, :]), # cos(yaw)
            np.sin(self.ref_path[0, :]), # cos(yaw)
            ])
        
        ## publishers
        self.publisher = self.create_publisher(DITrajectory, "nominal_traj", 10)
        self.publisher_viz = self.create_publisher(PoseArray, "nominal_traj/viz", 10)
        self.publisher_goal = self.create_publisher(PoseStamped, "goal_pose", 10)


        ## subs
        self.vicon_sub = self.create_subscription(TransformStamped,  "vicon/px4_1/px4_1", self.state_callback,1)

        ## timers
        self.timer_period = 0.5 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def state_callback(self, msg):

        self.x = msg.transform.translation.x
        self.y = msg.transform.translation.y
        self.z = msg.transform.translation.z
        self.yaw = quat2yaw(msg.transform.rotation)

    def estimate_current_phase(self):

        if self.x is None:
            return 0.0 # just go to the start

        delta  = self.ref - np.reshape(np.array([self.x, self.y, self.z, np.cos(self.yaw), np.sin(self.yaw)]), (5, 1))

        delta[2, :] = 0
        delta[3, :] = delta[3, :] * 0.2
        delta[4, :] = delta[4, :] * 0.2

        ind = np.argmin(np.linalg.norm(delta, axis=0))

        theta = self.ref_theta[ind]

        return theta


    def timer_callback(self):

        t_now = self.get_clock().now()

        phase0 = self.estimate_current_phase()

        if self.path_type == "circle":
            _, dt, path  = self.get_circle_path(phase0, self.horizon, 20)
        elif self.path_type == "figure8":
            _, dt, path = self.get_figure8_path(phase0, self.horizon, 20)

        else:
            self.get_logger().warn("NOT SUPPORTED PATH TYPE")
            return

        # traj msg
        traj_msg = self.make_msg(dt, path)
        traj_msg.header.stamp = t_now.to_msg()
        traj_msg.header.frame_id = "vicon/world"
        self.publisher.publish(traj_msg)

        # traj viz msg
        traj_msg_viz = PoseArray()
        traj_msg_viz.header = traj_msg.header
        traj_msg_viz.poses = traj_msg.poses
        self.publisher_viz.publish(traj_msg_viz)
        self.get_logger().info("pub..")

        # goal msg
        goal_msg = PoseStamped()
        goal_msg.header = traj_msg.header
        goal_msg.pose = traj_msg.poses[0]
        self.publisher_goal.publish(goal_msg)


    def get_circle_path(self, phase0, horizon, num=20):

        ts = np.linspace(0, horizon, num=num)
        dt = ts[1] - ts[0]

        thetas = phase0 + self.circle_omega * ts

        path = np.zeros((10, num))
        
        # actively add some yaw varation
        path[0, :] = thetas + np.pi/2  + (30.0 * np.pi / 180.0) * np.sin(10*thetas) # yaws

        # xyz
        path[1, :] = self.circle_x0 + self.circle_r0 * np.cos(thetas)
        path[2, :] = self.circle_y0 + self.circle_r0 * np.sin(thetas)
        path[3, :] = self.fly_at_z

        # vx, vy, vz
        path[4, :] = -self.circle_omega * self.circle_r0 * np.sin(thetas)
        path[5, :] = self.circle_omega * self.circle_r0 * np.cos(thetas)
        path[6, :] = 0.0

        # ax, ay, az
        path[7, :] = -self.circle_omega**2 * self.circle_r0 * np.cos(thetas)
        path[8, :] = -self.circle_omega**2 * self.circle_r0 * np.sin(thetas)
        path[9, :] = 0.0

        return thetas, dt, path

    def get_figure8_path(self, phase0, horizon, num=20):

        ts = np.linspace(0, horizon, num=num)
        dt = ts[1] - ts[0]
        thetas = self.figure8_omega * ts

        path = np.zeros((10, num))

        # xyz
        path[1, :] = self.figure8_x0 + self.figure8_rx * np.cos(thetas)
        path[2, :] = self.figure8_y0 + self.figure8_ry * np.sin(2*thetas)
        path[3, :] = self.fly_at_z

        # vx, vy, vz
        path[4, :] = -self.figure8_omega * self.figure8_rx * np.sin(thetas)
        path[5, :] = 2*self.figure8_omega * self.figure8_ry * np.cos(2*thetas)
        path[6, :] = 0.0 
        
        # ax, ay, az
        path[7, :] = -self.figure8_omega**2 * self.figure8_rx * np.cos(thetas)
        path[8, :] = -4*self.figure8_omega**2 * self.figure8_ry * np.sin(2*thetas)
        path[9, :] = 0.0

        # yaws
        path[0, :] = np.arctan2(vy, vx)

        return thetas, dt, path


    def make_msg(self, dt, path):

        # construct the message
        msg = DITrajectory()
        msg.dt = dt
        for i in range(path.shape[1]):

            pose = Pose()
            pose.position.x = path[1, i]
            pose.position.y = path[2, i]
            pose.position.z = path[3, i]
            pose.orientation = yaw2quat(path[0, i])

            msg.poses.append(pose)

            twist = Twist()
            twist.linear.x = path[4, i]
            twist.linear.y = path[5, i]
            twist.linear.z = path[6, i]
            
            msg.twists.append(twist)

            acc = Accel()
            acc.linear.x = path[7, i]
            acc.linear.y = path[8, i]
            acc.linear.z = path[9, i]

            msg.accelerations.append(acc)

        return msg


def main(args=None):
    
    rclpy.init(args=args)
    node = NominalPlanner2()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
