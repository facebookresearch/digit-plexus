
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/usr/bin/env python3
from typing import List
import numpy as np
import argparse

import rclpy
from rclpy.utilities import remove_ros_args
import rclpy.node
import rclpy.executors
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.msg import SetParametersResult, Parameter

from plexus.ros2.plexus_pkg.plexus_pkg.allegro_driver import AllegroDriver

class ROSAllegroController(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('allegro_controller')

        self.declare_parameter('can_interface', 'can0')
        try:
            self.robot = AllegroDriver(can_channel=self.get_parameter('can_interface').value)
        except RuntimeError as e:
            self.get_logger().fatal(str(e))
            raise e
        self.dt = 0.003
        self.dof = 16


        #controller gains
        for i in range(16):
            self.declare_parameter(f'gains_pd/{i}/p', 6.)
            self.declare_parameter(f'gains_pd/{i}/d', 0.15)
            self.declare_parameter(f'position_offset/{i}', 0.)
        self.declare_parameter(f'qv_alpha', 0.7)
        self.declare_parameter('torque_limit', 0.5)
        self.q_des = None

        # state
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", qos_profile_system_default)
        self.joint_target_sub = self.create_subscription(JointState, "joint_cmd", self.joint_cmd_cb, qos_profile_system_default)
        self.lib_cmd_sub = self.create_subscription(String, "lib_cmd", self.lib_cmd_cb, qos_profile_system_default)
        self.add_on_set_parameters_callback(self.param_cb)

        self.create_timer(self.dt, self.control_loop)

        self.q = None
        self.prev_q = None
        self.qv = None
        self.torque_des = None
        self.motor_status = None

        self.last_control = self.get_clock().now()

    def param_cb(self, param: List[Parameter]) -> SetParametersResult:
        for p in param:
            print(f'param update {p.name}: {p.value}')
        return SetParametersResult(successful=True)

    def lib_cmd_cb(self, msg: String) -> None:
        if msg.data == "home":
            q_home= np.zeros(16)
            q_home[12] = 0.4
            self._move_joint(q_home, torque_target=False)
        elif msg.data == "off":
            self._set_motor(False)

    def joint_cmd_cb(self, msg: JointState) -> None:
        if len(msg.position) > 0 and len(msg.effort) > 0:
            self.get_logger().error(f"received both position and effort commands, ignoring")
            return

        if len(msg.position) == 16:
            self._move_joint(msg.position, torque_target=False)
        elif len(msg.effort) == 16:
            self._move_joint(msg.effort, torque_target=True)
        else:
            self.get_logger().warning(f'invalid cmd: {msg}')

    def _set_motor(self, is_on: bool):
        if self.motor_status is None or self.motor_status != is_on:
            self.robot.set_motor(is_on)
            self.get_logger().info(f"motor switched {is_on}")
        self.motor_status = is_on 

    def _move_joint(self, target_cmd, torque_target=False):
        if torque_target:
            if self.q_des is not None: 
                self.get_logger().info(f"switching to torque mode")
            self.q_des = None
            self.torque_des = np.array(target_cmd)
        else:
            if self.torque_des is not None: 
                self.get_logger().info(f"switching to position mode")
            self.q_des = np.array(target_cmd)
            self.torque_des = None
        self.cmd_updatetime = self.get_clock().now()
        self._set_motor(True)

    def control_loop(self) -> None:

        # timing
        start_control = self.get_clock().now()
        interval = start_control - self.last_control
        self.last_control = start_control

        # getting parameters
        self.kp = np.array([self.get_parameter(f'gains_pd/{i}/p').value for i in range(self.dof)])
        self.kd = np.array([self.get_parameter(f'gains_pd/{i}/d').value for i in range(self.dof)])
        self.position_offset= np.array([self.get_parameter(f'position_offset/{i}').value for i in range(self.dof)])
        qv_alpha = self.get_parameter('qv_alpha').value

        if self.q is not None:
            self.prev_q = self.q 

        self.q = self.robot.get_joint_pos() + self.position_offset

        #initialize
        if self.prev_q is None:
            self.prev_q = self.q

        #velocity estimate
        qv_findiff = (self.q - self.prev_q) / (interval.nanoseconds /1e9)
        if self.qv is None:
            self.qv = qv_findiff
        else:
            self.qv = (1-qv_alpha) * self.qv + qv_alpha * qv_findiff

        if self.q_des is None and self.torque_des is None:
            self._move_joint(self.q, torque_target=False)        

        if self.torque_des is None:
            #PD
            err = self.q_des - self.q
            torque = self.kp * err - self.kd * self.qv
        else:
            torque = self.torque_des

        torque_limit = self.get_parameter('torque_limit').value
        torque = np.clip(torque, -torque_limit, torque_limit)
        self.robot.set_torque(torque)

        if (start_control - self.cmd_updatetime ).nanoseconds > 3_000_000_000:
            self._set_motor(False)

        #publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'allegro_joint_{i}.0' for i in range(16)]
        msg.position = self.q.tolist()
        msg.velocity = self.qv.tolist()
        msg.effort = torque.tolist()
        self.joint_state_publisher.publish(msg)

        # check timing 
        if interval.nanoseconds/1e9 > 2.0 * self.dt:
            self.get_logger().warning(f'slow control loop {interval.nanoseconds/1e9} > {self.dt}', throttle_duration_sec=1)

        control_time = self.get_clock().now() - start_control
        if control_time.nanoseconds/1e9 > self.dt * 0.9:
            self.get_logger().warn(f"control time {control_time.nanoseconds/1e9} near/over {self.dt}", throttle_duration_sec=1)


def main() -> None:
    rclpy.init()
    try:
        pdnode = ROSAllegroController()
        rclpy.spin(pdnode)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print("ERROR")


if __name__ == '__main__':
    main()
