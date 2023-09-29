import launch

def generate_launch_description():

    bagname = "run2"

    topics = [
            "/diagnostics/timing/process_sfc",
            "/diagnostics/timing/process_esdf",
            "/diagnostics/timing/process_depth",
            "/diagnostics/timing/planning_dmp",
            "/diagnostics/timing/gatekeeper",
            "/diagnostics/timing/gatekeeper_total",
            # "/diagnostics/timing/mpc",
            # "/diagnostics/timing/mpc_total",
            "/tf",
            "/goal_pose",
            "/camera/color/image_raw",
            "/nvblox_node/esdf_pointcloud",
            "/global_map",
            "/nvblox_node/esdfAABB_obs",
            "/nvblox_node/esdfAABB_unk",
            "/nvblox_node/sfc/viz",
            "/nominal_traj/viz",
            "/path_jps",
            "/committed_traj/viz",
            "/nvblox_node/sfc/chebyshev_center",
            "/virtual_obstacles",
            ]

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics + ["-o", bagname],
            cwd=["/workspaces/isaac_ros-dev/bags"],
            output='screen'
        )
    ])
                 
