import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    launch_noisy_node = False

    robot = "laptop_realsense_true"

    # Config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'config_vicon.yaml')

    vicon_node = Node(
        package="vicon_bridge",
        executable="vicon_bridge",
        parameters=[config_file], # dont forget to change the robot name in this config file too
        )

    # define some static tfs
    vicon_world_NED = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = [
                "--x", "0.0", 
                "--y", "0.0",
                "--z", "0.0",
                "--roll", f"{1.0 * math.pi}", 
                "--pitch", "0",
                "--yaw", f"{0.5 * math.pi}",
                "--child-frame-id", "/vicon/world/NED",
                "--frame-id", "/vicon/world"]
            )

    return LaunchDescription([
        vicon_node,
        vicon_world_NED,
        ])
