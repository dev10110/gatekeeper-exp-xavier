import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    robot = "laptop_realsense_true"

    # Config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'config_vicon.yaml')

    noisy_vicon_node = Node(
            package="vicon_bridge",
            executable="noisy_vicon",
            parameters=[config_file],
            remappings=[
                ("transform", f"vicon/{robot}/{robot}"),
                ("pose", f"pose"),
                ("pose_with_covariance", f"pose_cov"),
                ]
        )

    return LaunchDescription([
        noisy_vicon_node,
        ])
