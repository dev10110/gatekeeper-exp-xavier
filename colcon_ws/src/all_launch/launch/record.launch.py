import launch
import time

grab_color = False

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

    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")

    run_num = 1
    mode = "raw"
    # mode = "noisy_maps"
    # mode = "clean_maps"
    bagname = f"run_{run_num}_{mode}_{timestr}"


    # raw measurement mode
    if mode == "raw":
        topics = bagtypes_tf() + bagtypes_vicon_true() + bagtypes_color() + bagtypes_depth();


    # maps mode
    if mode == "clean_maps" or mode == "noisy_maps":
        topics = bagtypes_tf() + bagtypes_vicon_true() +bagtypes_vicon_noisy() + bagtypes_maps();

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics + ["-o", bagname],
            cwd=[f"/workspaces/isaac_ros-dev/rosbags/run_{run_num}"],
            output='screen'
        )
    ])
