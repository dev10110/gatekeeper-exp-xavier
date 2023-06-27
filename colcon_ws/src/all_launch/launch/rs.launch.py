import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'realsense.yaml')

    # Realsense driver node
    realsense_node = ComposableNode(
        namespace="camera",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[config_file])

    # Container
    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
        ],
        output='screen')

    return LaunchDescription([realsense_container])
