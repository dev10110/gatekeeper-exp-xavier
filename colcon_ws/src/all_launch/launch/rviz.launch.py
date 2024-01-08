import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
            
    file_names = [
      "esdf.rviz",
      "cesdf.rviz",
      "depth_camera.rviz",
      # "all_tsdf.rviz",
      # "cert_esdf.rviz",
      # "cert_tsdf_cert_dist.rviz",
      # "cert_tsdf_est_dist.rviz",
      # "cert_tsdf_correction.rviz",
      # "cert_tsdf_weight.rviz"
    ]

    procs = []

    for f in file_names:
       fp = os.path.join(get_package_share_directory("all_launch"), "rviz_config", f);


       proc = ExecuteProcess(
          cmd = ["rviz2", "-d", fp])
       procs.append(proc)

    return LaunchDescription(procs)
