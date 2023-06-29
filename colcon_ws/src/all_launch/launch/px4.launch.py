import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

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
            output=False)

    return LaunchDescription([
        microXRCE_bridge,
        mavlink
        ])
