# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    nvblox_bringup_dir = get_package_share_directory('all_launch')
    nav2_bringup_dir = get_package_share_directory('all_launch') # location of nav2 installation


    # Config file
    nav2_param_file = os.path.join(nvblox_bringup_dir,
                                   'config', 'nav2.yaml')

    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch2.py')),
        launch_arguments={'use_sim_time': 'False',
                          'params_file': nav2_param_file,
                          'autostart': 'True',
                          'use_composition': 'False',
                          }.items())

    return LaunchDescription([nav2_launch])
