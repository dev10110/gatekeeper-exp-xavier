

## Setup

1. Setup dev environment: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md, but change the ISAAC_ROS_WS variable as appropriate
2. Setup realsense: https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md
3. Inside the docker, run the `realsense-viewer` and make sure you are able to get images
4. Inside the docker, build the `colcon_ws`
```
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source ~/.bashrc
```

5. Change the launch script to avoid running `apt-get update` everytime you run `./scripts/run-dev.sh`

6. change to my custom Dockerfile:


## Useful other commands:
Restart docker:
```
sudo systemctl daemon-reload
sudo systemctl restart docker
```

Colcon build but in release mode:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
it will take longer, and recompile everything that was not built in release mode, but it will make the runtime code much faster


