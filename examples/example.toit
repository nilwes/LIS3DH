// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import lis3dh

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device lis3dh.I2C_ADDRESS
  sensor := lis3dh.Lis3dh device

  sensor.enable
  100.repeat:
    acc := sensor.read_acceleration
    print_ "$(%4.2f acc.x), $(%4.2f acc.y), $(%4.2f acc.z)" // Print to serial output.
    sleep --ms=20
  sensor.disable
