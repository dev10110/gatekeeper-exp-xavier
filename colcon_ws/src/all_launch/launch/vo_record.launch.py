import launch
import time
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription


def bagtypes_tf():
    return ["/tf", "/tf_static"]

def bagtypes_vicon_true():
    return [
            "/vicon/laptop_realsense_true/laptop_realsense_true"
            ]

def bagtypes_color():
    return [
                "/camera/color/image_raw",
                "/camera/color/camera_info",
                # "/camera/color/metadata"
            ]

def bagtypes_depth():
    return [
                # "/camera/depth/camera_info",
                # "/camera/aligned_depth_to_color/image_raw",
                # "/camera/realsense_splitter_node/output/depth"
                "/camera/realsense_splitter_node/output/depth",
                # "/camera/depth/image_rect_raw",
                # "/camera/depth/metadata"
                "/camera/aligned_depth_to_color/camera_info",
                ]

def bagtypes_stereo():
    return [
                "/camera/realsense_splitter_node/output/infra_1",
                "/camera/realsense_splitter_node/output/infra_2",
                "/camera/infra1/camera_info",
                "/camera/infra2/camera_info",
                # "/camera/infra1/metadata",
                # "/camera/infra2/metadata",

           ]

def bagtypes_imu():
    return [
            "/camera/imu"
            ]

def generate_launch_description():
    
    # grab the vicon launch file
    rs = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('all_launch'), 'launch', 'vo_rs.launch.py')]))

    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")

    bagname = f"raw_{timestr}"

    topics = bagtypes_tf() + bagtypes_vicon_true() + bagtypes_color() + bagtypes_depth() + bagtypes_stereo() + bagtypes_imu();

    return launch.LaunchDescription([
        rs, 
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics + ["-o", bagname] + ["-d", "60"], 
            cwd=[f"/workspaces/isaac_ros-dev/rosbags/vo"],
            output='screen'
        )
    ])
