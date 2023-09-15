import rclpy
from rclpy.node import Node
import numpy as np

from .utils_euler import yaw2quat

from dasc_msgs.msg import DITrajectory
from geometry_msgs.msg import Pose, Twist, Accel, PoseArray, PoseStamped


class NominalPlanner(Node):

    def __init__(self):

        super().__init__("nominal_planner")

        ## params

        ### general params
        self.path_type = "circle"
        self.horizon = 2.0 # seconds
        self.fly_at_z = 0.5 # meters
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
        
        ## publishers
        self.publisher = self.create_publisher(DITrajectory, "nominal_traj", 10)
        self.publisher_viz = self.create_publisher(PoseArray, "nominal_traj/viz", 10)
        self.publisher_goal = self.create_publisher(PoseStamped, "goal_pose", 10)


        ## subs

        ## timers
        self.timer_period = 0.5 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)



    def timer_callback(self):

        t_now = self.get_clock().now()
        tau_ns = (t_now - self.t0).nanoseconds
        tau_s = 1e-9 * tau_ns

        if self.path_type == "circle":
            traj_msg = self.get_circle_path(tau_s)
        elif self.path_type == "figure8":
            traj_msg = self.get_figure8_path(tau_s)

        else:

            self.get_logger().warn("NOT SUPPORTED PATH TYPE")
            return


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


    def get_circle_path(self, tau_s):

        ts = np.linspace(tau_s, tau_s + self.horizon, num=20)

        thetas = self.circle_omega * ts

        x = self.circle_x0 + self.circle_r0 * np.cos(thetas)
        y = self.circle_y0 + self.circle_r0 * np.sin(thetas)
        z = self.fly_at_z + 0.0 * x


        # actively add some yaw varation
        yaws = thetas + np.pi/2  + (30.0 * np.pi / 180.0) * np.sin(10*thetas)

        vx = -self.circle_omega * self.circle_r0 * np.sin(thetas)
        vy = self.circle_omega * self.circle_r0 * np.cos(thetas)
        vz = 0.0 * vx
        
        ax = -self.circle_omega**2 * self.circle_r0 * np.cos(thetas)
        ay = -self.circle_omega**2 * self.circle_r0 * np.sin(thetas)
        az = 0.0 * ax

        return self.make_msg(ts, x, y, z, yaws, vx, vy, vz, ax, ay, az)

    def get_figure8_path(self, tau_s):
        ts = np.linspace(tau_s, tau_s + self.horizon, num=20)

        thetas = self.figure8_omega * ts

        x = self.figure8_x0 + self.figure8_rx * np.cos(thetas)
        y = self.figure8_y0 + self.figure8_ry * np.sin(2*thetas)
        z = self.fly_at_z + 0.0 * x


        vx = -self.figure8_omega * self.figure8_rx * np.sin(thetas)
        vy = 2*self.figure8_omega * self.figure8_ry * np.cos(2*thetas)
        vz = 0.0 * vx
        
        ax = -self.figure8_omega**2 * self.figure8_rx * np.cos(thetas)
        ay = -4*self.figure8_omega**2 * self.figure8_ry * np.sin(2*thetas)
        az = 0.0 * ax
        
        yaws = np.arctan2(vy, vx)

        return self.make_msg(ts, x, y, z, yaws, vx, vy, vz, ax, ay, az)


    def make_msg(self, ts, x, y, z, yaws, vx, vy, vz, ax, ay, az):

        # construct the message
        msg = DITrajectory()
        msg.dt = ts[1] - ts[0]
        for i in range(len(ts)):

            pose = Pose()
            pose.position.x = x[i]
            pose.position.y = y[i]
            pose.position.z = z[i]
            pose.orientation = yaw2quat(yaws[i])

            msg.poses.append(pose)

            twist = Twist()
            twist.linear.x = vx[i]
            twist.linear.y = vy[i]
            twist.linear.z = vz[i]
            
            msg.twists.append(twist)

            acc = Accel()
            acc.linear.x = ax[i]
            acc.linear.y = ay[i]
            acc.linear.z = az[i]

            msg.accelerations.append(acc)

        return msg


def main(args=None):
    
    rclpy.init(args=args)
    node = NominalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
