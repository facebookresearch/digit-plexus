#!/usr/bin/env python
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Install
# python -m pip install python-can

# Protocol
# http://wiki.wonikrobotics.com/AllegroHandWiki/images/e/eb/V4_AllegroHandUsersManual_1.1.pdf

import argparse
import math
import struct
import time
from enum import Enum
from typing import Any, List

import can
import numpy as np
from numpy.typing import NDArray

"""
http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/CAN_Protocol_v4.0

0x20 (position requests)
 (000.000976)  can0  080   [0]  remote request
 (000.000079)  can0  084   [0]  remote request
 (000.000040)  can0  088   [0]  remote request
 (000.000059)  can0  08C   [0]  remote request
 (000.000044)  can0  080   [8]  CA FD BD FE 90 FF CE FF
 (000.000150)  can0  084   [8]  F9 FF CF FE 9A FF BE FF
 (000.000116)  can0  088   [8]  18 00 D3 FE A7 FF D6 FF
 (000.000174)  can0  08C   [8]  34 0A EF FF AF 00 FC FF
0x80 (info)
 (000.000143)  can0  200   [0]  remote request
 (000.000173)  can0  200   [7]  02 04 06 04 00 3C 05
0x38 (temp)
 (000.000241)  can0  0E0   [0]  remote request
 (000.000040)  can0  0E4   [0]  remote request
 (000.000039)  can0  0E8   [0]  remote request
 (000.000040)  can0  0EC   [0]  remote request
 (000.000040)  can0  0E0   [4]  2B 2A 29 25
 (000.000040)  can0  0E4   [4]  29 2B 29 27
 (000.000138)  can0  0E8   [4]  27 27 27 24
 (000.000081)  can0  0EC   [4]  30 26 24 F6
0x60 (set torque)
 (000.000209)  can0  180   [8]  02 00 CB 00 42 00 25 00
 (000.000079)  can0  184   [8]  01 00 B9 00 4A 00 33 00
 (000.000040)  can0  188   [8]  F1 FF C9 00 38 00 1F 00
 (000.000040)  can0  18C   [8]  58 02 05 00 8D FF FA FF
"""


class AllegroCommand(Enum):
    """
    https://github.com/simlabrobotics/allegro_hand_ros_v4/blob/master/src/allegro_hand_driver/src/candrv/candef.h
    """

    # Define CAN Command
    ID_CMD_SYSTEM_ON = 0x40
    ID_CMD_SYSTEM_OFF = 0x41
    ID_CMD_SET_TORQUE = 0x60
    ID_CMD_SET_TORQUE_1 = ID_CMD_SET_TORQUE + 0
    ID_CMD_SET_TORQUE_2 = ID_CMD_SET_TORQUE + 1
    ID_CMD_SET_TORQUE_3 = ID_CMD_SET_TORQUE + 2
    ID_CMD_SET_TORQUE_4 = ID_CMD_SET_TORQUE + 3
    ID_CMD_SET_POSE_1 = 0xE0
    ID_CMD_SET_POSE_2 = 0xE1
    ID_CMD_SET_POSE_3 = 0xE2
    ID_CMD_SET_POSE_4 = 0xE3
    ID_CMD_SET_PERIOD = 0x81
    ID_CMD_CONFIG = 0x68

    # Define CAN Data Reqeust (RTR)
    ID_RTR_HAND_INFO = 0x80
    ID_RTR_SERIAL = 0x88
    ID_RTR_FINGER_POSE = 0x20
    ID_RTR_FINGER_POSE_1 = ID_RTR_FINGER_POSE + 0
    ID_RTR_FINGER_POSE_2 = ID_RTR_FINGER_POSE + 1
    ID_RTR_FINGER_POSE_3 = ID_RTR_FINGER_POSE + 2
    ID_RTR_FINGER_POSE_4 = ID_RTR_FINGER_POSE + 3
    ID_RTR_IMU_DATA = 0x30
    ID_RTR_TEMPERATURE = 0x38
    ID_RTR_TEMPERATURE_1 = ID_RTR_TEMPERATURE + 0
    ID_RTR_TEMPERATURE_2 = ID_RTR_TEMPERATURE + 1
    ID_RTR_TEMPERATURE_3 = ID_RTR_TEMPERATURE + 2
    ID_RTR_TEMPERATURE_4 = ID_RTR_TEMPERATURE + 3

    @property
    def CAN_CMD_ID(self) -> int:
        return self.value << 2


class AllegroDriver:
    def __init__(self, can_channel: str = "can0", drain: bool = True) -> None:
        try:
            self.bus = can.Bus(
                interface="socketcan",
                channel=can_channel,
                can_filters=[
                    {
                        "can_id": AllegroCommand.ID_RTR_FINGER_POSE.CAN_CMD_ID,
                        "can_mask": 0xFF0,  # all finger pose 1111,1111,0000
                        "extended": False,
                    }  # get position
                ],
            )
        except OSError:
            self.bus = None
            raise RuntimeError(
                "CAN device not found, \
                    check if plexus USBC is plugged in, see trouble shooting here  \
                    https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html"
            )

        self.misc_bus = can.Bus(
            interface="socketcan",
            channel=can_channel,
            can_filters=[
                {
                    "can_id": AllegroCommand.ID_RTR_HAND_INFO.CAN_CMD_ID,
                    "can_mask": 0xFFC,
                    "extended": False,
                },  # get info (0x80<<2)
                {
                    "can_id": AllegroCommand.ID_RTR_TEMPERATURE.CAN_CMD_ID,
                    "can_mask": 0xFF0,  # all finger temp 1111, 1111, 0000
                    "extended": False,
                },  # get temp (0x38<<2 - 0x3B <<2)
                {
                    "can_id": AllegroCommand.ID_RTR_SERIAL.CAN_CMD_ID,
                    "can_mask": 0xFFC,
                    "extended": False,
                },  # get serial(0x88 <<2)
            ],
        )

        # properly shutdown hand that's still running
        self.set_period()
        while drain:
            msg = self.bus.recv(timeout=0.005)
            if msg is not None:
                if msg.is_error_frame:
                    raise RuntimeError("CAN receiving error frame, check allegro hand")
                elif not msg.is_rx:
                    raise RuntimeError(
                        "Another program sending can command, you might have another driver running"
                    )
                else:
                    print(f"draining packet {msg}")
            else:
                break

    def shutdown(self) -> None:
        try:
            if self.bus:
                self.set_torque([0.0] * 16)
                self.set_motor(False)
                self.set_period(0, 0, 0, 0)
        except Exception as e:
            print(str(e))
        finally:
            if self.bus:
                self.bus.shutdown()
                self.misc_bus.shutdown()

    def __del__(self) -> None:
        self.shutdown()

    def send_msg(
        self,
        msg_id: int,
        data: Any = None,
        bus: can.Bus = None,
        is_remote_frame: bool = False,
    ) -> None:
        msg = can.Message(
            arbitration_id=msg_id << 2,
            data=data,
            is_remote_frame=is_remote_frame,
            is_extended_id=False,
        )

        if bus is None:
            bus = self.bus

        try:
            bus.send(msg)
        except can.exceptions.CanOperationError as e:
            print(e)
            raise RuntimeError(
                "can bus present, but not ready - did you turn on allegrohand"
            )

    def send_rtr_request(
        self, bus: can.Bus, msg_id: int, timeout: float = 0.01
    ) -> can.Message:
        self.send_msg(msg_id, is_remote_frame=True, bus=bus)
        msg = bus.recv(timeout)
        if msg is None:
            raise TimeoutError()
        assert (
            msg_id == msg.arbitration_id >> 2
        ), f"received remote frame \
            allegro msg_id {hex(msg.arbitration_id>>2)}(can msg_id = {hex(msg.arbitration_id)}>>2) \
            not match requested allegro msg_id {hex(msg_id)}"
        return msg

    def get_serial(self) -> str:
        msg = self.send_rtr_request(self.misc_bus, AllegroCommand.ID_RTR_SERIAL.value)
        return msg.data.decode().rstrip("\x00")

    def get_temp(self) -> List[int]:
        temp: List[int] = []
        for finger in range(4):
            msg_id = AllegroCommand.ID_RTR_TEMPERATURE.value + finger
            msg = self.send_rtr_request(self.misc_bus, msg_id=msg_id)
            assert msg.arbitration_id >> 2 == msg_id, "received wrong packet"
            try:
                temp += struct.unpack("4B", msg.data)
            except struct.error as e:
                print(msg)
                raise e
        return temp

    def get_joint_pos(self) -> NDArray:
        temp: List[float] = []
        for finger in range(4):
            msg = self.send_rtr_request(
                self.bus, msg_id=AllegroCommand.ID_RTR_FINGER_POSE.value + finger
            )
            temp += struct.unpack("4h", msg.data)
        return np.array(temp) * math.pi / 180 * 333 / 65536

    def get_info(self) -> dict:
        msg = self.send_rtr_request(
            self.misc_bus, msg_id=AllegroCommand.ID_RTR_HAND_INFO.value
        )
        hw_ver, fw_ver, is_left_hand, palm_temp, stat = struct.unpack("hhbBB", msg.data)
        return {
            "hw_ver": hex(hw_ver),
            "sw_ver": hex(fw_ver),
            "hand": "left" if is_left_hand else "right",
            "palm_temp": palm_temp,
            "stat": hex(stat),
            "motor": bool(stat & 0x1),
            "joint_temp_fault": bool(stat & 0x2),
            "joint_temp_throttle": bool(stat & 0x4),
            "joint_comm_fault": bool(stat & 0x8),
            "palm_temp_fault": bool(stat & 0x10),
        }

    def set_motor(self, is_on: bool) -> None:
        print(f"motor {is_on}")
        if is_on:
            self.send_msg(msg_id=AllegroCommand.ID_CMD_SYSTEM_ON.value)
        else:
            self.send_msg(msg_id=AllegroCommand.ID_CMD_SYSTEM_OFF.value)

    def set_torque(self, torqueN: List[float]) -> None:
        # https://github.com/0wu/ros-allegro/blob/ros2-main/ros-allegro/allegro_hand_driver/src/AllegroHandDrv.cpp#L70C1-L71C1
        # PWM Range (-1200, 1200)
        assert len(torqueN) == 16
        torquePWM = list(map(lambda x: int(min(max(-1.0, x), 1.0) * 1200.0), torqueN))
        for finger in range(4):
            data = struct.pack("4h", *torquePWM[finger * 4 : finger * 4 + 4])
            self.send_msg(
                msg_id=AllegroCommand.ID_CMD_SET_TORQUE.value + finger, data=data
            )

    def set_period(
        self, joint_pos: int = 0, imu: int = 0, temp: int = 0, status: int = 0
    ) -> None:
        data = struct.pack("HHHH", joint_pos, imu, temp, status)
        self.send_msg(msg_id=AllegroCommand.ID_CMD_SET_PERIOD.value, data=data)


def test_torque(can_channel: str) -> None:
    print("==== Testing {} ====".format(can_channel))
    robot = AllegroDriver(can_channel=can_channel)
    t = time.perf_counter()
    for i in range(3):
        print(robot.get_temp())
        print(time.perf_counter() - t)
    print(robot.get_serial())

    robot.set_torque([0] * 16)
    robot.set_motor(True)
    for i in range(3):
        robot.set_torque(
            [
                -0.0,
            ]
            * 16
        )
        with np.printoptions(
            linewidth=100,
            formatter={"float": lambda x: np.format_float_positional(x, precision=2)},
        ):
            print("hand status", robot.get_info())
            print("joint position", robot.get_joint_pos())
        time.sleep(0.003)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--can_channel", default=None, type=str, help="can channel, empty for all"
    )
    args = parser.parse_args()

    if args.can_channel is None:
        interfaces = can.detect_available_configs(interfaces="socketcan")
        for iface in interfaces:
            test_torque(iface["channel"])
    else:
        test_torque(args.can_channel)


if __name__ == "__main__":
    main()
