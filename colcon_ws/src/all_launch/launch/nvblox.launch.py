import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction   
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (Node, ComposableNodeContainer, SetParameter,
                                SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    bringup_dir = get_package_share_directory('all_launch')
    base_config_dir = os.path.join(bringup_dir, 'config')

    # Config files
    base_config = os.path.join(base_config_dir, 'config_nvblox.yaml')
    
    rs_config = os.path.join(base_config_dir, 'config_realsense.yaml')

    # Realsense driver node
    realsense_node = ComposableNode(
        namespace="",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[rs_config])

    # Nvblox node
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode')

    # Nvblox node container
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # realsense_node,
            nvblox_node,
            ],
        output='screen')

    group_action = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(base_config),
        SetParameter(name='global_frame',
                     value=LaunchConfiguration('global_frame', default='vicon/world')),

        # Remappings for realsense data
        SetRemap(src=['depth/image'],
                 dst=['camera/depth/image_rect_raw']),
        SetRemap(src=['depth/camera_info'],
                 dst=['camera/depth/camera_info']),
        SetRemap(src=['color/image'],
                 dst=['camera/color/image_raw']),
        SetRemap(src=['color/camera_info'],
                 dst=['camera/color/camera_info']),

        # Include the node container
        nvblox_container
    ])


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
        group_action, 
        static_tf
        ])
