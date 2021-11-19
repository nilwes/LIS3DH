// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import i2c
import ..src.LIS3DH

main:
  acc := []
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  rate := 10
  
  device := bus.device I2C_ADDRESS
  sensor := lis3dh device

  sensor.enable --max_g_force = 2 --output_data_rate = rate
  100.repeat:
    acc = sensor.read_acceleration
    print_ "$(%5.2f acc[0]), $(%5.2f acc[1]), $(%5.2f acc[2])"
    sleep --ms=(1000/rate).to_int // Read data the same rate as they are produced
  sensor.disable