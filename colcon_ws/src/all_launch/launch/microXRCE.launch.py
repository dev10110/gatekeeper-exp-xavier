from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    bridge = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB1', '-b', '921600'], 
            name='microXRCEAgent', 
            output='both')

    return LaunchDescription([bridge])
