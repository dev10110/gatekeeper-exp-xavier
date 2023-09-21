import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (Node, ComposableNodeContainer, SetParameter,
                                SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode

import numpy as np


def generate_launch_description():

    use_sim_time = False
    use_gk = True
    use_jps = True

    # launch the jps
    jps_node = Node(
            name="jps_planner_2d",
            executable="jps_planner_2d",
            package="jps3d_ros",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"replan_rate_hz": 0.5},
                {"quad_radius_m": 0.10},
                {"desired_speed_m_per_s": 0.25},
                {"resample_rate_hz": 5.0}, 
                {"smoothing_param": 0.2},
                {"dmp_potential_radius_m": 0.2}
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
                {"terminal_yaw_freq_hz": 1.0 / 10.0},
                {"terminal_yaw_amplitude_rad": -90.0 * np.pi / 180.0},
                {"terminal_z_freq_hz": 0.0},
                {"terminal_z_amplitude_m": 0.0 },
                ]
            )

    nodes = [
            setpoint_pub_node
            ]

    if use_gk:
        nodes.append(gatekeeper)

    if use_jps:
        nodes.append(jps_node)
    # else:
    #     nodes.append(nom_planner)

    return LaunchDescription(nodes)
