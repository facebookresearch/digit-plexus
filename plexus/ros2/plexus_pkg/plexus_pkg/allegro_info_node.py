

# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/usr/bin/env python3
from typing import List
import numpy as np
import argparse
import signal
from multiprocessing import Process

import rclpy
from rclpy.utilities import remove_ros_args
import rclpy.logging
import rclpy.node
import rclpy.executors
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.msg import SetParametersResult, Parameter
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

from plexus_pkg.allegro_driver import AllegroDriver

class ROSAllegroInfo(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('allegro_info')
        self.status_publisher = self.create_publisher(DiagnosticArray, "status", qos_profile_system_default)
        self.temp_publisher = self.create_publisher(Float32MultiArray, "joint_temperature", qos_profile_system_default)
        self.declare_parameter('can_interface', 'can0')
        try:
            self.robot = AllegroDriver(can_channel=self.get_parameter('can_interface').value, drain=False)
        except RuntimeError as e:
            self.get_logger().fatal(str(e))
            raise e

        self.create_timer(1, self.info_loop)

    def info_loop(self):
        info = self.robot.get_info()
        temp = self.robot.get_temp()

        diagmsg = DiagnosticArray()
        diagmsg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        diagmsg.status.append(status)
        status.name = "status"
        status.values=[KeyValue(key=k, value=str(v)) for k, v in info.items()]

        status = DiagnosticStatus()
        status.name = 'joint_temp'
        diagmsg.status.append(status)
        status.values=[KeyValue(key=f'joint_temp_{i}', value=str(v)) for i, v in enumerate(temp)]

        self.status_publisher.publish(diagmsg)
        self.temp_publisher.publish(Float32MultiArray(data=temp))


def main() -> None:
    rclpy.init()
    infonode= ROSAllegroInfo()
    try:
        rclpy.spin(infonode)
    except KeyboardInterrupt as e:
        rclpy.logging.get_logger(__file__).info('Ctrl-C received, shutting down')

if __name__ == '__main__':
    main()
