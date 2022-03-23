// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
Driver for the LIS3DH accelerometer.

Datasheet: https://www.st.com/resource/en/datasheet/lis3dh.pdf
*/

import binary
import math
import serial.device as serial
import serial.registers as serial

I2C_ADDRESS     ::= 0x18
I2C_ADDRESS_ALT ::= 0x19

/**
Driver for the LIS3DH accelerometer.
*/
class Lis3dh:
  static RATE_1HZ   ::= 1
  static RATE_10HZ  ::= 2
  static RATE_25HZ  ::= 3
  static RATE_50HZ  ::= 4
  static RATE_100HZ ::= 5
  static RATE_200HZ ::= 6
  static RATE_400HZ ::= 7

  static RANGE_2G  ::= 0
  static RANGE_4G  ::= 1
  static RANGE_8G  ::= 2
  static RANGE_16G ::= 3

  // Device Registers.
  static WHO_AM_I_        ::=  0x0F

  static CTRL_REG1_       ::=  0x20
  static CTRL_REG4_       ::=  0x23
  static CTRL_REG5_       ::=  0x24

  static OUT_X_L_         ::=  0x28
  static OUT_X_H_         ::=  0x29
  static OUT_Y_L_         ::=  0x2A
  static OUT_Y_H_         ::=  0x2B
  static OUT_Z_L_         ::=  0x2C
  static OUT_Z_H_         ::=  0x2D

  static CHIP_ID_         ::=  0x33

  static GRAVITY_STANDARD_ ::= 9.80665

  // The currently selected range.
  range_/int := 0

  reg_/serial.Registers ::= ?

  constructor dev/serial.Device:
    sleep --ms=5 // Sensor startup time according to datasheet
    reg_ = dev.registers
    // Check chip ID
    if (reg_.read_u8 WHO_AM_I_) != CHIP_ID_: throw "INVALID_CHIP"

  /**
  Enables the sensor.
  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE_1HZ
  - $RATE_10HZ
  - $RATE_25HZ
  - $RATE_50HZ
  - $RATE_100HZ
  - $RATE_200HZ
  - $RATE_400HZ

  The $range parameter defines the maximum +/- range (in g).
  Valid values for $range are:
  - $RANGE_2G: +-2G (19.61 m/s²)
  - $RANGE_4G: +-4G (39.23 m/s²)
  - $RANGE_8G: +-8G (78.45 m/s²)
  - $RANGE_16G: +-16G (156.9 m/s²)
  */
  enable -> none
      --rate/int = RATE_10HZ
      --range/int = RANGE_2G:

    if not RATE_1HZ <= rate <= RATE_400HZ: throw "INVALID_RANGE"
    // 8.8. CTRL1.
    rate_bits := rate << 4

    // We always enable all three axes.
    axes_bits := 0b111

    ctrl1 := rate_bits | axes_bits

    if not RANGE_2G <= range <= RANGE_16G: throw "INVALID_RANGE"
    range_ = range
    range_bits := range << 4
    high_resolution_bit := 0b1000

    ctrl4 := range_bits | high_resolution_bit

    reg_.write_u8 CTRL_REG1_ ctrl1
    reg_.write_u8 CTRL_REG4_ ctrl4

    sleep --ms=7 // Wait 7 Mhz to give the sensor time to wake up.

  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
    to restart the accelerometer.
  */
  disable:
    // Fundamentally we only care for the rate-bits: as long as they
    // are 0, the device is disabled.
    // It's safe to change the other bits as well.
    reg_.write_u8 CTRL_REG1_ 0x00

  /**
  Reads the acceleration on the x, y and z axis.
  The returned values are in in m/s².
  */
  read_acceleration -> math.Point3f:
    AUTO_INCREMENT_BIT ::= 0b1000_0000

    x := reg_.read_i16_le (OUT_X_L_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_L_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_L_ | AUTO_INCREMENT_BIT)

    // Section 2.1, table4:
    // The linear acceleration sensitivity depends on the range.
    // We are always in high-resolution mode:
    // - RANGE_2G:   1mg/LSB
    // - RANGE_4G:   2mg/LSB
    // - RANGE_8G:   4mg/LSB
    // - RANGE_16G:  12mg/LSB   // <- Note that the 16G sensitivity is not 8mg/LSB as expected.
    SENSITIVITIES ::= #[1, 2, 4, 12]
    sensitivity := SENSITIVITIES[range_]
    x *= sensitivity
    y *= sensitivity
    z *= sensitivity

    // The 16.0 is because the values are only 12 bits long.
    factor := GRAVITY_STANDARD_ / 1000.0 / 16.0  // Constant folded because it's one expression.

    return math.Point3f
        x * factor
        y * factor
        z * factor

