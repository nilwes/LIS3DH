// Copyright (C) 2023 Toitware ApS.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import lis3dh

SDA ::= 21
SCL ::= 22
INT1 ::= 12

main:
  interrupt := gpio.Pin INT1 --input

  bus := i2c.Bus
    --sda=gpio.Pin SDA
    --scl=gpio.Pin SCL

  device := bus.device lis3dh.I2C_ADDRESS
  sensor := lis3dh.Lis3dh device

  sensor.enable
      --rate=lis3dh.Lis3dh.RATE_100HZ
      --detect_wake_up

  while true:
    interrupt.wait_for 1
    print "movement detected"
    sleep --ms=1000
    sensor.read_interrupt_cause

  sensor.disable
