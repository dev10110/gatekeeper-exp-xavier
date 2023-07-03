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
        'config', 'realsense.yaml')

    # Realsense driver node
    realsense_node = ComposableNode(
        namespace="camera",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[config_file])
    
    # decompros node
    decomp_node = ComposableNode(
            namespace="camera", 
            package="decomp_ros",
            plugin="decompros::SeedDecomp",
            name="seedDecomp_component",
            parameters=[
                {"fov_v": 58.0},
                {"fov_h": 77.0},
                {"fov_range": 4.0},
                {"fov_obs_ray_range": 2.0},
                {"fov_obs_spacing": 0.25},
                {"fov_obs_skip_first": 1},
                {"publish_fov_obstacles": False}
                ],
            remappings=[
                ("cloud_in", "depth/color/points"),
                ]
            )

    decomp_ros_viz = ComposableNode(
            namespace="camera",
            package="decomp_ros",
            plugin="decompros::VizPoly"
            )

    # Container
    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            decomp_node,
            decomp_ros_viz
        ],
        output='screen')

    off_x = "0.0"
    off_y = "0"
    off_z = "0"
    off_roll = str(math.pi)
    off_pitch = "0"
    off_yaw = "0"
    static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
               off_x, off_y, off_z, off_yaw, off_pitch, off_roll, "vicon/px4_1/px4_1", "camera_link"]
            )

    return LaunchDescription([
        realsense_container, 
        static_tf
        ])
