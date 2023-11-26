import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'config_realsense.yaml')

    # Realsense driver node
    realsense_node = ComposableNode(
        namespace="/camera",
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

    off_x = "0.0"
    off_y = "0.0"
    off_z = "0.0"
    off_roll = str(math.pi)
    off_pitch = "0"
    off_yaw = "0"
    static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
               "--x", off_x,
               "--y", off_y,
               "--z", off_z,
               "--yaw", off_yaw,
               "--pitch", off_pitch,
               "--roll", off_roll,
               "--frame-id", "vicon/laptop_realsense/laptop_realsense",
               "--child-frame-id", "camera_link"
               ]
            )

    return LaunchDescription([
        realsense_container, 
        static_tf
        ])
