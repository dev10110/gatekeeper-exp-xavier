import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (Node, ComposableNodeContainer, SetParameter,
                                SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    use_sim_time = False
    use_gk = True

    # launch the path planning
    jps = Node(
            name="jps_planner_2d",
            executable="jps_planner_2d",
            package="jps3d_ros",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"replan_rate_hz": 0.5},
                {"quad_radius_m": 0.15},
                {"desired_speed_m_per_s": 0.4},
                {"resample_rate_hz": 5.0}
            ]
            )

    nom_planner = Node(
            name="nominal_planner",
            executable="nominal_planner",
            package="nominal_planner",
            parameters = [
                {"fly_at_z_m": 1.0},
                ]
            )


    # filter the trajectory
    gatekeeper = Node(
            name="gatekeeper",
            package="gatekeeper",
            executable="gatekeeper",
            parameters=[
                {"use_sim_time": use_sim_time}, 
                {"fly_at_z_m": 1.0},
                ],  
            output="both"
            )

    # actually track the trajectory
    traj_topic = "/committed_traj" if use_gk else "/nominal_traj"
    setpoint_pub_node = Node(
            name="setpoint_publisher",
            package="dasc_ros_utils",
            executable="setpointPublisher",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"trajectory_topic": traj_topic},
                {"publish_rate_hz": 50.0},
                ]
            )

    nodes = [
            # jps,
            # nom_planner,
            setpoint_pub_node
            ]

    if use_gk:
        nodes.append(gatekeeper)

    return LaunchDescription(nodes)
