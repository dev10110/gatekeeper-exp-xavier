import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():

    # grab the vicon launch file
    vicon = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('all_launch'), 'launch', 'vicon.launch.py')]))

    # Config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'config_realsense2.yaml')

    # Realsense driver node
    realsense_node = ComposableNode(
        namespace="",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[config_file])

    # splitter
    realsense_splitter_node = ComposableNode(
        namespace="camera",
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA'
                    }],
        remappings=[('input/infra_1', '/camera/infra1/image_rect_raw'),
                    ('input/infra_1_metadata', '/camera/infra1/metadata'),
                    ('input/infra_2', '/camera/infra2/image_rect_raw'),
                    ('input/infra_2_metadata', '/camera/infra2/metadata'),
                    ('input/depth', '/camera/aligned_depth_to_color/image_raw'),
                    ('input/depth_metadata', '/camera/depth/metadata'),
                    ('input/pointcloud', '/camera/depth/color/points'),
                    ('input/pointcloud_metadata', '/camera/depth/metadata'),
                    ])
    
    # Container
    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            realsense_splitter_node,
           ],
        output='screen')

    off_x = "0.0"
    off_y = "0.0"
    off_z = "0.0"
    off_roll = "0.0" 
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
               "--frame-id", "vicon/laptop_realsense_true/laptop_realsense_true",
               "--child-frame-id", "camera_link"
               ]
            )

    return LaunchDescription([
        vicon,
        realsense_container, 
        static_tf
        ])
