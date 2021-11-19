// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// LIS3DH data sheet: https://www.st.com/resource/en/datasheet/lis3dh.pdf

import binary
import serial.device as serial
import serial.registers as serial

I2C_ADDRESS     ::= 0x18

class lis3dh:
  //Device Registers
  static WHO_AM_I_        ::=  0x0F
 
  static CTRL_REG1_       ::=  0x20
  static CTRL_REG4_       ::=  0x23
  static CTRL_REG5_       ::=  0x24
  static STATUS_REG_      ::=  0x27
  static FIFO_CTRL_REG_   ::=  0x2E

  static OUT_X_L_         ::=  0x28
  static OUT_X_H_         ::=  0x29
  static OUT_Y_L_         ::=  0x2A
  static OUT_Y_H_         ::=  0x2B
  static OUT_Z_L_         ::=  0x2C
  static OUT_Z_H_         ::=  0x2D

  static CHIP_ID_              ::=  0x33 
  static LP_ENABLE_XYZ_        ::=  0b0000_0111
  static NO_FILTER_            ::=  0b00_00_0_0_00
  static NO_INTERRUPTS_        ::=  0b0000_0000
  static CTRL_REG4_CONFIG_     ::=  0b0_0_00_0_00_0 // Noblock data, Little Endian, Full scale, High Res disabled, No self Test, SPI interface 4-wire
  static CTRL_REG5_CONFIG_     ::=  0b1_1_00_0000   // Turn on FIFO
  static FIFO_MODE_            ::=  0b01_0_00000    // FIFO mode
  static STREAM_MODE_          ::=  0b10_0_00000    // Stream mode

  static GRAVITY_STANDARD_ ::= 9.80665 

  static LSB16_TO_KILO_LSB10_   ::=  64000

  static ODR_ ::= {
    0      : 0b0000_0000,
    1      : 0b0001_0000,
    10     : 0b0010_0000,
    25     : 0b0011_0000,
    50     : 0b0100_0000,
    100    : 0b0101_0000,
    200    : 0b0110_0000,
    400    : 0b0111_0000,
    } 

  static FULL_SCALE_G_ ::= {
    2      : 0b00_00_0000,
    4      : 0b00_01_0000,
    8      : 0b00_10_0000,
    16     : 0b00_11_0000,
  }

  full_scale_  := 0

  reg_/serial.Registers ::= ?

  constructor dev/serial.Device:
    sleep --ms=5 // Sensor startup time according to datasheet
    reg_ = dev.registers
    // Check chip ID
    if (reg_.read_u8 WHO_AM_I_) != CHIP_ID_: throw "INVALID_CHIP"

  /**
  Enables the sensor.
  The $output_data_rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - 1  Hz
  - 10 Hz
  - 25 Hz
  - 50 Hz
  - 100 Hz
  - 200 Hz
  - 400 Hz

  The $max_g_force parameter defines the maximum +/- range (in g).
  Valid values for $max_g_force are:
  - 2 g
  - 4 g
  - 8 g
  - 16 g
  */
  enable -> none
      --output_data_rate/int = 10 
      --max_g_force/int = 2: 

    full_scale_ = max_g_force
    reg_.write_u8 CTRL_REG1_ (ODR_[output_data_rate] | LP_ENABLE_XYZ_)
    reg_.write_u8 CTRL_REG4_ FULL_SCALE_G_[max_g_force]
    reg_.write_u8 CTRL_REG5_ CTRL_REG5_CONFIG_ // Enable FIFO
    reg_.write_u8 FIFO_CTRL_REG_ STREAM_MODE_ // STREAM mode
  
  /**
  Reads the acceleration on the x, y and z axis.
  The returned values are in in m/s².
  */
  read_acceleration -> List:
    lsb_value_ := 0
    x_ := 0
    y_ := 0
    z_ := 0

    33.repeat: // Read entire FIFO buffer + 1 to get newest acceleration value
      x_ = ((reg_.read_i8 OUT_X_H_) << 8) | (reg_.read_u8 OUT_X_L_)
      y_ = ((reg_.read_i8 OUT_Y_H_) << 8) | (reg_.read_u8 OUT_Y_L_)
      z_ = ((reg_.read_i8 OUT_Z_H_) << 8) | (reg_.read_u8 OUT_Z_L_)

    // This scaling process is adopted from Adafruit LIS3DH driver 
    // at https://github.com/adafruit/Adafruit_LIS3DH/blob/master/Adafruit_LIS3DH.cpp
    if full_scale_ == 2:  lsb_value_ = 4
    if full_scale_ == 4:  lsb_value_ = 8
    if full_scale_ == 8:  lsb_value_ = 16;
    if full_scale_ == 16: lsb_value_ = 48;
    x_acc_ := (lsb_value_ * (x_.to_float / LSB16_TO_KILO_LSB10_)) * GRAVITY_STANDARD_
    y_acc_ := (lsb_value_ * (y_.to_float / LSB16_TO_KILO_LSB10_)) * GRAVITY_STANDARD_
    z_acc_ := (lsb_value_ * (z_.to_float / LSB16_TO_KILO_LSB10_)) * GRAVITY_STANDARD_  

    return [x_acc_ , y_acc_, z_acc_]

  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
  to restart the accelerometer.
  */
  disable:
    reg_.write_u8 CTRL_REG1_ 0x00