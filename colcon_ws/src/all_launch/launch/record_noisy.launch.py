import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory





def generate_launch_description():

    mode  = "noisy" 

    if mode=="noisy":
        print("noisy mode")
        epsilon_R = 0.0006
        epsilon_t = 0.0005
        save_path = "/workspaces/isaac_ros-dev/maps/run_1/noisy"
    elif mode=="clean":
        print("clean mode")
        epsilon_R = 0.0;
        epsilon_t = 0.0;
        save_path = "/workspaces/isaac_ros-dev/maps/run_1/clean"
    else: 
        print("unsupported mode")
        return LaunchDescription()

    rosbag_play_file = "/workspaces/isaac_ros-dev/rosbags/mapping/run_1/run_1_raw_2024_01_10-12_04_18"

    all_launch_dir = get_package_share_directory('all_launch')
    
    # launch the recording node 
    record = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(all_launch_dir, 'launch', 'record.launch.py')),
        launch_arguments={
            'raw_mode': 'False', 
            'clean_mode': 'True' if mode == "clean" else "False",
            'noisy_mode': 'True' if mode == "noisy" else "False",
        }.items()
    )

    # noisy vicon node
    robot = 'laptop_realsense_true'
    noisy_vicon = Node(
                package="vicon_bridge",
                executable="noisy_vicon",
                parameters=[
                    { 'update_rate_hz': 30.0},
                    {'epsilon_R_per_frame': epsilon_R},
                    {'epsilon_t_per_frame': epsilon_t}],
                remappings=[
                    ("transform", f"vicon/{robot}/{robot}"),
                    ("pose", f"pose"),
                    ("pose_with_covariance", f"pose_cov"),
                    ]
            )

    # nvblox node
    nvblox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(all_launch_dir, 'launch', 'nvblox.launch.py'))
            )


    # rviz
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(all_launch_dir, 'launch', 'rviz.launch.py'))
            )

    # save service
    service = Node(
            package="save_ply",
            executable="client",
            parameters=[
                {"save_path": save_path},
                {"save_time_seconds": 105.0}
                ]
            )

    # rosbag play
    play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_play_file],
            output='screen'
        )


    return LaunchDescription( [
        record,
        service,
        noisy_vicon,
        nvblox,
        rviz,
        play
        ])
