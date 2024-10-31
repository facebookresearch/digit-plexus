
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/usr/bin/env python
import os
import sys
import threading
import rclpy
from sensor_msgs.msg import MagneticField
from plexus.ros2.plexus_pkg.msg import PowerStatus
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from plexus.ros2.plexus_pkg.plexus_pkg.plexus import Plexus

def main(port) -> None:
    # python -m serial.tools.list_ports -v

    plexus_obj = Plexus(port)
    node=rclpy.create_node('sensor_patch')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    pubs = {}
    def pub(topic, msg):
        msg.header.stamp=node.get_clock().now().to_msg()
        if topic in pubs:
            pub = pubs[topic]
        else:
            pub = node.create_publisher(type(msg), topic, rclpy.qos.ReliabilityPolicy.RELIABLE)
            pubs[topic]=pub
        pub.publish(msg)

    while plexus_obj.is_open() and rclpy.ok():
        data = plexus_obj.read()
        if data is None:
            node.get_logger().warn("plexus receive a None packet, serial port timeout")
            continue
        if data.data is not None:
            topic = f"reskin/finger{data.data.finger}/link{data.data.link}/sensor{data.data.sensor_id}"
            msg = MagneticField(
                magnetic_field = Vector3(x=data.data.mag.x, y=data.data.mag.y, z=data.data.mag.z)
            )
            pub(topic, msg)

        elif data.status.power_status is not None:
            pub(f"plexus/power_status", PowerStatus(
                    voltage=data.status.power_status.voltage,
                    current=data.status.power_status.current,
                    power=data.status.power_status.power,
                ))
        else:
            node.get_logger().warn(f"unknown message type {data}, please add in {__file__}")


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    # Check if /dev/ttyACM0 exists

    default_device = '/dev/ttyPlexus'
    if os.path.exists(default_device):
        main(port=default_device)
    else:
        main(port='/dev/ttyACM0')
    sys.exit()
