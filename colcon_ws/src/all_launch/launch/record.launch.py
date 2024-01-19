import launch
import time
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def bagtypes_tf():
    return ["/tf", "/tf_static"]

def bagtypes_vicon_true():
    return [
            "/vicon/laptop_realsense_true/laptop_realsense_true"
            ]

def bagtypes_vicon_noisy():
    return [
            "/vicon/laptop_realsense/laptop_realsense"
            ]

def bagtypes_color():
    return [
                "/camera/color/image_raw",
                "/camera/color/camera_info",
                "/camera/color/metadata"
            ]

def bagtypes_depth():
    return [
                "/camera/depth/camera_info",
                "/camera/depth/image_rect_raw",
                "/camera/depth/metadata"
                ]

def bagtypes_maps():
    return [
            "/nvblox_node/map_slice",
            "/nvblox_node/certified_map_slice",
            "/nvblox_node/esdf_pointcloud",
            "/nvblox_node/certified_esdf_pointcloud",
            ]

def generate_launch_description():
    
    raw_mode = LaunchConfiguration('raw_mode')
    raw_mode_arg  = DeclareLaunchArgument(
            'raw_mode', default_value="False", 
            description="Set true if you want to record a raw bag file")
    
    noisy_mode = LaunchConfiguration('noisy_mode')
    noisy_mode_arg  = DeclareLaunchArgument(
            'noisy_mode', default_value="False",
            description="Set true if you want to record a noisy bag file")
    
    clean_mode = LaunchConfiguration('clean_mode')
    clean_mode_arg  = DeclareLaunchArgument(
            'clean_mode', default_value="False",
            description="Set true if you want to record a clean bag file")
    
    run_num = 2
    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")

    raw_bagname = f"run_{run_num}_raw_{timestr}"
    noisy_bagname = f"run_{run_num}_noisy_{timestr}"
    clean_bagname = f"run_{run_num}_clean_{timestr}"


    # topics 
    raw_topics = bagtypes_tf() + bagtypes_vicon_true() + bagtypes_color() + bagtypes_depth();
    clean_topics = bagtypes_tf() + bagtypes_vicon_true() +bagtypes_vicon_noisy() + bagtypes_maps();
    noisy_topics = bagtypes_tf() + bagtypes_vicon_true() +bagtypes_vicon_noisy() + bagtypes_maps();

    raw_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + raw_topics + ["-o", raw_bagname], 
            cwd=[f"/workspaces/isaac_ros-dev/rosbags/mapping/run_{run_num}"],
            output='screen',
            condition = IfCondition(raw_mode)
            )
    noisy_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + noisy_topics + ["-o", noisy_bagname, "--use-sim-time"],
            cwd=[f"/workspaces/isaac_ros-dev/rosbags/mapping/run_{run_num}"],
            output='screen',
            condition = IfCondition(noisy_mode)
            )
    clean_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + clean_topics + ["-o", clean_bagname, "--use-sim-time"],
            cwd=[f"/workspaces/isaac_ros-dev/rosbags/mapping/run_{run_num}"],
            output='screen',
            condition = IfCondition(clean_mode)
            )

    ## 

    return launch.LaunchDescription([
        raw_mode_arg,
        clean_mode_arg,
        noisy_mode_arg,
        raw_record, 
        noisy_record,
        clean_record
    ])
