import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    microXRCE_bridge = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB1', '-b', '921600'], 
            name='microXRCEAgent', 
            output='both')

    mavlink_router_conf_file = os.path.join(
            get_package_share_directory('all_launch'), 'config', 'mavlink-router.conf')

    mavlink = ExecuteProcess(
            cmd = ["mavlink-routerd", "-c", mavlink_router_conf_file],
            name="mavlink-routerd",
            output='log')
    
    robot_name = "px4_1"
    vicon_px4_bridge_node = Node(
        package='vicon_px4_bridge', executable='bridge', output='screen',
        parameters=[{'px4_name': robot_name, 'vicon_name': robot_name}]
    )

    vicon_launch  = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("all_launch"), "/launch", "/vicon.launch.py"])
            )

    return LaunchDescription([
        microXRCE_bridge,
        mavlink,
        vicon_px4_bridge_node, 
        vicon_launch
        ])
