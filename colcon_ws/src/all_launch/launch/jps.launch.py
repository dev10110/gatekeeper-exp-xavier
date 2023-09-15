
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

    # launch the path planning
    jps = Node(
            name="jps_planner_2d",
            executable="jps_planner_2d",
            package="jps3d_ros",
            parameters=[
                {"use_sim_time": True},
                {"replan_rate_hz": 2.0},
                {"quad_radius_m": 0.15},
                {"desired_speed_m_per_s": 1.0},
                {"resample_rate_hz": 5.0}
            ]
            )

    return LaunchDescription([
        jps,
        ])
