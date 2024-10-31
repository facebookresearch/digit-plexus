
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the metahand node
    metahand_node = Node(
        package="plexus_pkg",
        executable="sensor_patch.py",
        name="plexus_hand",
        output="screen"
    )

    # Return the LaunchDescription with the metahand node
    return LaunchDescription([metahand_node])

