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
  static WHO_AM_I_         ::= 0x0F

  static CTRL_REG1_        ::= 0x20
  static CTRL_REG2_        ::= 0x21
  static CTRL_REG3_        ::= 0x22
  static CTRL_REG4_        ::= 0x23
  static CTRL_REG5_        ::= 0x24

  static REFERENCE_        ::= 0x26

  static INT1_CFG_         ::= 0x30
  static INT1_SRC_         ::= 0x31
  static INT1_THS_         ::= 0x32
  static INT1_DURATION_    ::= 0x33

  static OUT_X_L_          ::= 0x28
  static OUT_X_H_          ::= 0x29
  static OUT_Y_L_          ::= 0x2A
  static OUT_Y_H_          ::= 0x2B
  static OUT_Z_L_          ::= 0x2C
  static OUT_Z_H_          ::= 0x2D

  static CHIP_ID_          ::= 0x33

  static GRAVITY_STANDARD_ ::= 9.80665

  static INTERRUPT_X_LOW_BIT  ::= 1
  static INTERRUPT_X_HIGH_BIT ::= 2
  static INTERRUPT_Y_LOW_BIT  ::= 4
  static INTERRUPT_Y_HIGH_BIT ::= 8
  static INTERRUPT_Z_LOW_BIT  ::= 16
  static INTERRUPT_Z_HIGH_BIT ::= 32

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
  Deprecated. Use $(enable --latch_interrupt) instead.
  */
  enable -> none
      --rate/int = RATE_10HZ
      --range/int = RANGE_2G
      --detect_free_fall/bool = false
      --latch_free_fall/bool
      --free_fall_duration/Duration = (Duration --ms=30):
    enable
        --rate=rate
        --range=range
        --detect_free_fall
        --free_fall_duration=free_fall_duration
        --latch_interrupt = latch_free_fall

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

  If $detect_free_fall is true, the sensor will raise INT1 (a pin on the
    sensor) when it detects a free fall.

  It is recommended to use a higher frequency (like $RATE_100HZ or higher) when
    free-fall detection is enabled. The frequency must be high enough so that
    two measurements fall within the $free_fall_duration duration.

  If $detect_free_fall is true and $latch_interrupt is set, then the
    INT1 pin will be high until it is cleared with $read_interrupt_cause.
    Without $latch_interrupt, the INT1 pin is only high for the duration of
    the free fall.

  The $free_fall_duration sets the minimum duration of a free-fall event for it
    to be recognized. The $rate must be high enough to allow for multiple sensor
    readings within the given duration.

  If $detect_wake_up is true then the sensor will raise INT1 when it detects
    movement.

  Both the $detect_free_fall and $detect_wake_up use the $detection_threshold
    to determine when they should trigger. The $detection_threshold is in
    mg, and the range depends on the changes $range.
    For $RANGE_2G (respectively $RANGE_4G, $RANGE_8G, and $RANGE_16G) the
    granularity is 16mg (respectively 32mg, 62mg, and 186mg). Note that
    the granularities don't scale linearly with the range.
  */
  enable -> none
      --rate/int = RATE_10HZ
      --range/int = RANGE_2G
      --detect_free_fall/bool = false
      --free_fall_duration/Duration = (Duration --ms=30)
      --detect_wake_up/bool = false
      --detection_threshold/int = (detect_free_fall ? 372 : 64)
      --latch_interrupt/bool = true:

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

    ctrl2 := 0x0  // Default value. High-pass filter disabled.
    ctrl3 := 0x0  // Default value. Might be changed by free-fall or movement detection below.
    ctrl4 := range_bits | high_resolution_bit
    ctrl5 := 0x0 // Default value

    int1_threshold := 0x0 // Default value
    int1_duration := 0x0 // Default value
    int1_cfg := 0x0 // Default value

    needs_reference := false

    if detect_free_fall and detect_wake_up:
      throw "Can't detect free fall and wake up at the same time."

    if detect_free_fall:
      // See chapter 6.4 of the sensor's application note:
      // https://www.st.com/resource/en/application_note/an3308-lis3dh-mems-digital-output-motion-sensor-ultralowpower-highperformance-3axis-nano-accelerometer-stmicroelectronics.pdf

      // Enable interrupt 1 to use the interrupt generator (int1_cfg).
      ctrl3 |= 0b100_0000  // IA1 interrupt on INT1.

      if free_fall_duration.in_ns <= 0: throw "INVALID_RANGE"

      // Depending on the frequency of the chip, which is set in ctr1.
      if rate == RATE_1HZ: int1_duration = free_fall_duration.in_s
      else if rate == RATE_10HZ: int1_duration = free_fall_duration.in_ms / 100
      else if rate == RATE_25HZ: int1_duration = free_fall_duration.in_ms / 40
      else if rate == RATE_50HZ: int1_duration = free_fall_duration.in_ms / 20
      else if rate == RATE_100HZ: int1_duration = free_fall_duration.in_ms / 10
      else if rate == RATE_200HZ: int1_duration = free_fall_duration.in_ms / 5
      else if rate == RATE_400HZ: int1_duration = (free_fall_duration * 4).in_ms / 10
      else: unreachable

      if int1_duration == 0: throw "Invalid rate/duration combination"

      // AOI and 6D are set to 10: "AND combination of interrupt events".
      // See Table 55. Interrupt mode.
      interrupt_mode := 0b1000_0000
      // Trigger the interrupt when each of the directions (x, y, z) are lower
      // than the threshold.
      interrupt_on_low := 0b010101

      int1_cfg |= interrupt_mode | interrupt_on_low

    if detect_wake_up:
      // Enable the high-pass filter.
      // This is optional for x/y wake-up but pretty much necessary for z wake-up since
      // the earth's gravity is always present.
      ctrl2 |= 0b1001  // Enable high-pass filter.
      // Enable interrupt 1 to use the interrupt generator (int1_cfg).
      ctrl3 |= 0b100_0000  // Interrupt activity 1 driven to INT1 pad.
      int1_duration = 0     // It should already be 0, but be explicit.

      // 6-direction movement recognition.
      interrupt_mode := 0b0100_0000
      // Trigger on all directions higher than the threshold.
      interrupt_on_high := 0b101010
      int1_cfg |= interrupt_mode | interrupt_on_high
      int1_cfg = 0x2A
      needs_reference = true  // We are using a high-pass filter, so we need to set the reference.

    if latch_interrupt and (detect_free_fall or detect_wake_up):
      ctrl5 |= 0b1000  // Latch interrupt request on INT1_SRC.

    if detect_free_fall or detect_wake_up:
      // The threshold value depends on the range.
      // The least significant bit in the register is worth:
      // - 16mg @ 2G
      // - 32mg @ 4G
      // - 62mg @ 8G
      // - 186mg @ 16G
      // We want to detect free fall at roughly 350mg.
      if range == RANGE_2G:
        int1_threshold = detection_threshold >> 4
      else if range == RANGE_4G:
        int1_threshold = detection_threshold >> 3
      else if range == RANGE_8G:
        int1_threshold = detection_threshold / 62
      else if range == RANGE_16G:
        int1_threshold = detection_threshold / 186
      else:
        unreachable

    reg_.write_u8 CTRL_REG1_ ctrl1
    reg_.write_u8 CTRL_REG2_ ctrl2
    reg_.write_u8 CTRL_REG3_ ctrl3
    reg_.write_u8 CTRL_REG4_ ctrl4
    reg_.write_u8 CTRL_REG5_ ctrl5

    reg_.write_u8 INT1_THS_ int1_threshold
    reg_.write_u8 INT1_DURATION_ int1_duration
    if needs_reference: reg_.read_u8 REFERENCE_
    reg_.write_u8 INT1_CFG_ int1_cfg

    sleep --ms=7 // Wait 7ms to give the sensor time to wake up.

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

  /**
  Clears the sensor's interrupt.

  When the sensor is configured to detect free-fall events, and the `--latch_free_fall`
    was provided during $enable, then the interrupt pin stays set until this function
    is called.

  Deprecated. Use $read_interrupt_cause instead.
  */
  clear_free_fall_interrupt:
    reg_.read_u8 INT1_SRC_

  /**
  Reads the interrupt cause and clears the interrupt.

  When the sensor is configured to detect free-fall or wake-up events, and the
    `--latch_interupt` was provided during $enable, then the interrupt pin stays set
    until this function is called.

  The returned integer has the $INTERRUPT_X_LOW_BIT set to high if a low event
    on the x axis was detected. The same goes for the y and z axis. Similarly,
    the $INTERRUPT_X_HIGH_BIT is set to high if a high event was detected.
  */
  read_interrupt_cause -> int:
    return reg_.read_u8 INT1_SRC_
