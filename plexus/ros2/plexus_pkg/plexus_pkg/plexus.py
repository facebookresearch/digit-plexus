# Copyright (c) Meta Platforms, Inc. and affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from enum import IntEnum
from typing import Optional, Union
import serial
from cobs import cobs
import plexus.ros2.plexus_pkg.plexus_pkg.plexus_pb_v1 as ppb

class Control:
    # axis control
    class Axis:
        X_AXIS_ENABLE = True
        X_AXIS_DISABLE = False
        Y_AXIS_ENABLE = True
        Y_AXIS_DISABLE = False
        Z_AXIS_ENABLE = True
        Z_AXIS_DISABLE = False

    # odr control
    class ODR(IntEnum):
        ODR_200HZ = 3
        ODR_100HZ = 4
        ODR_50HZ = 5
        ODR_25HZ = 6
        ODR_12_5HZ = 7
        ODR_6_25HZ = 8

    # mode control
    class Mode(IntEnum):
        MODE_AVG_NONE = 0
        MODE_AVG_2 = 1
        MODE_AVG_4 = 2
        MODE_AVG_8 = 3

    # reset control
    class Reset(IntEnum):
        SOFT_MAGNETIC = 0
        HARD_MAGNETIC = 1
        POWER_ON = 2
        POWER_OFF = 3
        POWER_CYCLE = 4


class Plexus(Control):
    def __init__(
        self, serial_port: str = None, read_timeout: Optional[int] = None
    ) -> None:
        self.__dev: serial.Serial = None
        self.serial_port: str = serial_port
        self.connect(read_timeout)

    def connect(self, read_timeout: Optional[int] = None) -> None:
        self.__dev = serial.Serial(self.serial_port, timeout=read_timeout)
        if not self.__dev.is_open:
            raise IOError(f"Could not access plexus at: {self.serial_port}")

    def read(self, as_dict: bool = False) -> Union[ppb.Plexus, dict]:
        data = self.__dev.read_until(b"\x00")
        data = self._decode(data)
        if as_dict:
            data = data.to_dict()
        return data

    def is_open(self) -> bool:
        return self.__dev.is_open

    def disconnect(self) -> None:
        self.__dev.close()
        self.__dev.reset_input_buffer()
        self.__dev.reset_output_buffer()

    def _send(self, data: ppb.Plexus) -> None:
        self.__dev.write(bytes(data))
        self.__dev.flush()

    def _decode(self, serialized_data: bytes) -> ppb.Plexus:
        cobs_data = serialized_data[:-1]
        mh_data = None
        try:
            pb_data = cobs.decode(cobs_data)
            mh_data = ppb.Plexus().parse(pb_data)
        except cobs.DecodeError:
            self.__dev.reset_input_buffer()

        return mh_data

    def control_axis(self, x_en: bool, y_en: bool, z_en: bool) -> None:
        cmd = ppb.Plexus(
            control=ppb.Control(axis=ppb.AxisControl(x_en, y_en, z_en))
        )
        self._send(cmd)

    def control_odr(self, odr: Control.ODR, mode: Control.Mode) -> None:
        cmd = ppb.Plexus(control=ppb.Control(odr=ppb.OdrControl(odr, mode)))
        self._send(cmd)

    def control_reset(self, reset_type: Control.Reset) -> None:
        cmd = ppb.Plexus(
            control=ppb.Control(
                reset=ppb.ResetControl(ppb.ResetControlResetType(reset_type))
            )
        )
        self._send(cmd)

    def status(self, show_only_missing: bool = True) -> None:
        cmd = ppb.Plexus(
            status=ppb.Status(
                sensor_status=ppb.SensorStatus(
                    ppb.Finger.INDEX if show_only_missing else ppb.Finger.NONE,
                    1,
                    1,
                    True,
                )
            )
        )
        self._send(cmd)