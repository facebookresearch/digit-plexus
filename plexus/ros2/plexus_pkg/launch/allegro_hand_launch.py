
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from plexus.ros2.plexus_pkg.plexus_pkg.allegro_driver import AllegroDriver
import can
from typing import Dict

def get_allegro_configs() -> Dict[str, Path]:
    launch_root = os.path.dirname(os.path.abspath(__file__))
    interfaces=can.detect_available_configs(interfaces="socketcan")
    configs = {}
    for iface in interfaces:
        allegro_driver = AllegroDriver(can_channel=iface["channel"])
        allegro_serial = allegro_driver.get_serial().lower()
        allegro_config_serial = Path(f"{launch_root}/allegro_node_params_{allegro_serial}.yaml")
        allegro_config_default= Path(f"{launch_root}/allegro_node_params.yaml")
    
        if allegro_config_serial.exists():
            print(f'using hand specific config - {allegro_config_serial.absolute()}')
            configs[iface["channel"]] = allegro_config_serial
        else:
            print(f'using default config - {allegro_config_default.absolute()}')
            configs[iface["channel"]] = allegro_config_default

    return configs


def _generate_launch_description(context):
    configs = get_allegro_configs()
    nodes = []
    for i, (can_interface, config) in enumerate(configs.items()):
        _ns = f"allegroHand{i}" if i > 0 else "allegroHand"
        allegro_controller_node  = Node(
            namespace=_ns,
            package="plexus_pkg",
            executable="allegro_controller_node.py",
            name="allegro_controller",
            parameters=[config, {"can_interface": can_interface}],
            respawn=True
        )
        nodes.append(allegro_controller_node)

        allegro_info_node  = Node(
            namespace=_ns,
            package="plexus_pkg",
            executable="allegro_info_node.py",
            name="allegro_info",
            parameters=[config, {"can_interface": can_interface}],
        )
        nodes.append(allegro_info_node)


        shutdown_action = RegisterEventHandler(
            OnProcessExit(
                target_action=allegro_controller_node,
                on_exit=[
                    LogInfo(msg="critical node failure"),
                    EmitEvent(event=Shutdown(
                        reason='critical node failure'))
                ]
            )
        )
        nodes.append(shutdown_action)
    return nodes

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_generate_launch_description)])